/**
 * @file trd_pc.cpp
 * @author qingyu
 * @brief PC（自瞄）链路线程 — USB CDC ACM 单一 owner：收 PC 自瞄数据转下板 + 发 IMU 数据给 PC
 * @version 1.0
 * @date 2026-09-23
 *
 * ## 数据流（两个方向都只在本线程里搬运）
 *
 *     PC ─USB→ 组帧/CRC → inter_cmd::PostFrame(StateAutoAim) ─→ user_can2_msgq
 *              ─→ mcu_inter 聚合(置 VisionOk/VisionFire) ─→ CAN ─→ 下板
 *
 *     PC ←USB─ PCSendAutoAimData ← pub_imu_to（框架 IMU 状态通道，轮询取最新值）
 *
 * ## 为什么收发在同一个线程（而不是两个）
 *
 *   · `usb::Usb::Send()` 非阻塞：未配置/端点在忙直接返回 false，发不出去只丢帧，
 *     不会把"收"卡住；接收完全由 USB 中断 + 环形缓冲驱动，线程只负责搬运；
 *   · USB 的"已配置 / 掉线 / 总线复位"是整条链路共享的状态，收发的判活必须用同一份；
 *   · 数据本身已经解耦：出向是 pull（pub_imu_to），入向是 push（PostFrame 队列）。
 *     真需要拆线程的条件是"发送变成阻塞式或需要重传窗口"，现在不满足。
 *
 * ## PC 掉线时的行为（本线程 + mcu_inter 两侧配合，阈值都是 100ms）
 *
 *   · 本线程：连续 kLinkTimeoutMs 没有 CRC 通过的帧 → 判链路失效：
 *       ① 不再投递自瞄数据；② 发给 PC 的帧里 mode 回 0（空闲），PC 侧据此知道上板没在用；
 *       ③ 清掉半截帧缓冲，等 PC 重连后重新同步（不需要握手）。
 *   · mcu_inter：按自瞄分片的"年龄"判活，超时就把 CommState.flags 的
 *       VisionOk / VisionFire 清零 → 下板 AutoAimValid/开火请求同时失效 → 回手动瞄准。
 *     这样即使 PC 拔线、PC 程序卡死、或上板本线程没来得及反应，下板也不会拿着陈旧
 *     自瞄角继续打。
 *   · 恢复：只要有新的 CRC 通过帧就自动恢复（数据自带有效性位），不需要重新握手。
 *
 * ## 待接入（不影响当前功能）
 *
 *   · bullet_speed：取 pub_from_body（下板实测弹速），下板还没发布时恒 0；
 *   · bullet_count：取发射机构线程的累计进弹次数，没开 CONFIG_TRD_BOOSTER 时恒 0；
 *   · 上板是否要用遥控自瞄开关（autoaim_ctrl）再门控一次，待定；当前不做门控。
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/Pc"

#include "thread.hpp"
#include "Init_entry.hpp"
#include "pc_protocol.hpp"
#include "inter_cmd.hpp"
#include "imu_to.hpp"
#include "usb.hpp"

#if defined(CONFIG_DUST_TPC_FROM_BODY)
#include "from_body.hpp"        // bullet_speed 来源（下板实测弹速）
#endif
#if defined(CONFIG_TRD_BOOSTER)
#include "trd_booster.hpp"      // bullet_count 来源（累计进弹次数）
#endif

#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(trd_pc, LOG_LEVEL_INF);

namespace thread::pc {

static Thread<3072> thread_ {};
static usb::Usb usb_ {};

// ==================== 链路参数 ====================
static constexpr uint32_t kPeriodMs      = 1;      // 主循环节拍（等 USB 接收通知的超时）
static constexpr uint32_t kTxMinPeriodMs = 10;     // 兜底周期：没有新 IMU 样本也发一帧（PC 侧判活）
/// PC 自瞄链路判活阈值：必须与 mcu_inter 的 kVisionTimeoutMs 一致
static constexpr uint32_t kLinkTimeoutMs = 100;
static constexpr uint8_t  kInitRetry     = 5;      // USB 初始化重试次数
static constexpr uint32_t kInitRetryMs   = 100;
static constexpr uint32_t kDropLogPeriod = 1000;   // 坏帧/丢帧累计到该值上报一次
static constexpr uint16_t kRxChunk       = 128;    // 每次从 USB 环形缓冲取的字节数

static_assert(pc_proto::kRecvFrameLen <= 64, "recv frame exceeds rx buffer");
static_assert(pc_proto::kRecvFrameLen <= kRxChunk, "recv frame larger than one read chunk");

// ==================== 本线程私有状态 ====================

/// 接收组帧缓冲（定长帧，够一帧即可）
static uint8_t  g_rx_buf[64] {};
static uint16_t g_rx_len = 0;

static uint32_t g_rx_frames = 0;        ///< 通过校验并投递出去的自瞄帧
static uint32_t g_rx_bad    = 0;        ///< CRC/范围检查失败或投递失败的帧
static uint32_t g_tx_fail   = 0;        ///< 发给 PC 的帧被丢弃次数
static uint32_t g_last_valid_ms = 0;    ///< 最后一次收到有效自瞄帧的时刻
static uint8_t  g_last_mode = pc_proto::kRecvModeIdle;   ///< 最后一帧有效自瞄数据里的 mode
static bool     g_link_ok = false;

/// 出向数据源：IMU 快照（pull，无订阅者，version 判新）
static topic::imu_to::Message g_imu {};
static uint32_t g_imu_ver      = 0;
static uint32_t g_last_sent_ver = 0;
static uint32_t g_last_send_ms  = 0;

// ==================== 接收：组帧 + 校验 + 转下板 ====================

/**
 * @brief 解析一帧 PC 数据（已确认帧头、长度正确）
 *
 * 校验顺序：CRC16 → mode 合法 + 浮点有限 + 量程 → 投递。
 * 任何一步失败都只丢这一帧（不改变链路状态：链路判活看的是"有效帧到达"）。
 */
static void DecodeRecv(const uint8_t *frame)
{
    const uint16_t crc_rx =
        static_cast<uint16_t>(frame[pc_proto::kRecvFrameLen - 2]) |
        static_cast<uint16_t>(static_cast<uint16_t>(frame[pc_proto::kRecvFrameLen - 1]) << 8);
    if (crc_rx != pc_proto::Crc16(frame, pc_proto::kRecvFrameLen - 2)) {
        if ((++g_rx_bad % kDropLogPeriod) == 1u) {
            LOG_WRN("pc frame crc err (total %u)", static_cast<unsigned>(g_rx_bad));
        }
        return;
    }

    PCRecvAutoAimData d {};
    memcpy(&d, frame, sizeof(d));

    if (!pc_proto::RecvSane(d)) {
        if ((++g_rx_bad % kDropLogPeriod) == 1u) {
            LOG_WRN("pc frame range err mode=%u (total %u)",
                    static_cast<unsigned>(d.mode), static_cast<unsigned>(g_rx_bad));
        }
        return;
    }

    // mode：0 空闲（不下发）/ 1 自瞄不开火 / 2 自瞄开火
    uint8_t flags = 0;
    if (d.mode >= pc_proto::kRecvModeAim) {
        flags = inter_cmd::kCommFlagVisionOk;
        if (d.mode == pc_proto::kRecvModeAimFire) {
            flags = static_cast<uint8_t>(flags | inter_cmd::kCommFlagVisionFire);
        }
    }

    // 只给下板用：走已预留的 push 通道（端到端队列，唯一消费者是 mcu_inter）
    inter_cmd::AutoAimState aim {};
    memcpy(&aim.yaw_angle[0],   &d.yaw.yaw_ang,     sizeof(float));
    memcpy(&aim.yaw_omega[0],   &d.yaw.yaw_vel,     sizeof(float));
    memcpy(&aim.yaw_acc[0],     &d.yaw.yaw_acc,     sizeof(float));
    memcpy(&aim.pitch_angle[0], &d.pitch.pitch_ang, sizeof(float));
    memcpy(&aim.pitch_omega[0], &d.pitch.pitch_vel, sizeof(float));
    memcpy(&aim.pitch_acc[0],   &d.pitch.pitch_acc, sizeof(float));

    if (!inter_cmd::PostFrame(inter_cmd::FrameType::StateAutoAim, aim, flags)) {
        // 发送线程来不及取（正常不会发生，队列深度 16 + 1ms 消费）：丢帧但保留链路判活
        if ((++g_rx_bad % kDropLogPeriod) == 1u) {
            LOG_WRN("auto-aim queue full (total %u)", static_cast<unsigned>(g_rx_bad));
        }
    } else {
        ++g_rx_frames;
    }

    g_last_valid_ms = k_uptime_get_32();
    g_last_mode     = d.mode;
}

/**
 * @brief 字节流入组帧状态机（定长帧：找双字节帧头 → 收满一帧 → 解析）
 */
static void FeedRx(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; ++i)
    {
        const uint8_t b = data[i];

        if (g_rx_len == 0) {
            if (b == pc_proto::kHead0) {
                g_rx_buf[g_rx_len++] = b;
            }
            continue;
        }

        if (g_rx_len == 1) {
            if (b == pc_proto::kHead1) {
                g_rx_buf[g_rx_len++] = b;
            } else if (b != pc_proto::kHead0) {
                g_rx_len = 0;                       // 帧头半截不对：丢掉这个 'S'
            }
            continue;
        }

        g_rx_buf[g_rx_len++] = b;
        if (g_rx_len >= pc_proto::kRecvFrameLen) {
            DecodeRecv(g_rx_buf);
            g_rx_len = 0;                           // 定长帧：整帧消费掉再重新找帧头
        }
    }
}

// ==================== 链路判活 ====================

static void UpdateLink()
{
    const uint32_t now = k_uptime_get_32();
    const bool ok = (g_last_valid_ms != 0u) && ((now - g_last_valid_ms) <= kLinkTimeoutMs);
    if (ok == g_link_ok) {
        return;
    }

    g_link_ok = ok;
    g_rx_len  = 0;                                  // 掉线/恢复都清半截帧，重新同步
    if (ok) {
        LOG_INF("PC link online");
    } else {
        LOG_INF("PC link LOST -> auto-aim data stops forwarding (lower board -> manual)");
    }
}

// ==================== 发送：IMU → PC ====================

/// 轮询 IMU 状态通道（读忙/无新样本时沿用上一帧）
static void PollImu()
{
    topic::imu_to::Message latest {};
    if (zbus_chan_read(&pub_imu_to, &latest, K_NO_WAIT) != 0) {
        return;
    }
    if (latest.version == 0u || latest.version == g_imu_ver) {
        return;
    }
    g_imu_ver = latest.version;
    g_imu     = latest;
}

/// 子弹速度：下板实测（未接入时恒 0）
static float BulletSpeed()
{
#if defined(CONFIG_DUST_TPC_FROM_BODY)
    static topic::from_body::Message body {};
    topic::from_body::Message latest {};
    if (zbus_chan_read(&pub_from_body, &latest, K_NO_WAIT) == 0) {
        body = latest;
    }
    return body.online ? body.shooter.bullet_speed : 0.0f;
#else
    return 0.0f;
#endif
}

/// 累计发弹数：发射机构线程按"进弹脉冲"计数（未接弹丸检测，是近似值）
static uint16_t BulletCount()
{
#if defined(CONFIG_TRD_BOOSTER)
    return static_cast<uint16_t>(atomic_get(&instance::booster::shot_count));
#else
    return 0u;
#endif
}

/**
 * @brief 组一帧 PCSendAutoAimData 发给 PC
 * @return true = 已提交给 USB 端点；false = 没配置好/端点忙（下一拍重试）
 */
static bool SendToPc()
{
    if (!usb_.IsReady()) {
        return false;
    }

    PCSendAutoAimData f {};

    // 回显上板当前是否在用 PC 的自瞄数据（链路新鲜 + 最近一帧不是"空闲"）
    f.mode = (g_link_ok && g_last_mode >= pc_proto::kRecvModeAim) ? pc_proto::kSendModeAim
                                                                 : pc_proto::kSendModeIdle;

    for (uint8_t i = 0; i < 4; ++i) {
        f.q[i] = g_imu.quaternion[i];
    }
    f.yaw.yaw_ang     = g_imu.yaw_total;     // 累计偏航（可跨圈）：世界系连续角，自瞄解算用
    f.yaw.yaw_vel     = g_imu.gyro[2];       // z 轴角速度 → 偏航角速度
    f.pitch.pitch_ang = g_imu.pitch;
    f.pitch.pitch_vel = g_imu.gyro[1];       // y 轴角速度 → 俯仰角速度
    f.bullet.bullet_speed = BulletSpeed();
    f.bullet.bullet_count = BulletCount();
    f.crc16 = pc_proto::FrameCrc(f);

    if (!usb_.Send(reinterpret_cast<const uint8_t *>(&f), sizeof(f))) {
        if ((++g_tx_fail % kDropLogPeriod) == 1u) {
            LOG_WRN("usb tx drop (not configured / ep busy), total %u",
                    static_cast<unsigned>(g_tx_fail));
        }
        return false;
    }
    return true;
}

/// 发送时机：IMU 出现新样本，或到了兜底周期（PC 掉线、IMU 没数据时也要让 PC 能判活）
static bool TxDue(uint32_t now)
{
    return (g_imu_ver != g_last_sent_ver) || ((now - g_last_send_ms) >= kTxMinPeriodMs);
}

// ==================== 主循环 ====================

static void Task(void*, void*, void*)
{
    for (;;)
    {
        const int64_t tick_start = k_uptime_ticks();

        // printk("pc communication\r\n");
        // ① 收：USB 中断已把字节压进环形缓冲，这里等通知（超时 = 主循环节拍）
        (void)k_sem_take(&usb_.sem_, K_MSEC(kPeriodMs));
        uint8_t chunk[kRxChunk];
        uint16_t n;
        while ((n = usb_.Read(chunk, sizeof(chunk))) > 0) {
            FeedRx(chunk, n);
        }
        // ② 链路判活（掉线/恢复只在沿上打日志）
        UpdateLink();

        // ③ 发：IMU 新样本优先，否则按兜底周期（心跳）
        PollImu();
        const uint32_t now = k_uptime_get_32();
        if (TxDue(now) && SendToPc()) {
            g_last_sent_ver = g_imu_ver;
            g_last_send_ms  = now;
        }

        const int64_t period = k_ms_to_ticks_ceil64(kPeriodMs);
        const int64_t used   = k_uptime_ticks() - tick_start;
        if (used < period) {
            k_sleep(K_TICKS(period - used));
        }
    }
}

// ==================== 初始化 ====================

bool thread_init()
{
    // 语义 alias：哪个 USB 是 PC（板级 overlay 提供）
    UsbHal::Config cfg {};
    cfg.busid    = 0;
    cfg.reg_base = DT_REG_ADDR(DT_ALIAS(pc_usb));
    cfg.irq_num  = DT_IRQN(DT_ALIAS(pc_usb));

    for (uint8_t retry = 0; retry < kInitRetry; ++retry)
    {
        if (usb_.Init(cfg)) {
            LOG_INF("pc usb ready (cdc acm)");
            return true;
        }
        k_msleep(kInitRetryMs);
    }

    LOG_ERR("pc usb init failed");
    return false;
}

bool thread_start()
{
    if (!usb_.IsReady()) {
        return false;
    }
    thread_.Start(Task, ThreadPrio::High);
    return true;
}

REGISTER_INIT  (thread_init,  PreInit,    High, "pc_init");
REGISTER_THREAD(thread_start, LateThread, High, "pc_start");

} // namespace thread::pc
