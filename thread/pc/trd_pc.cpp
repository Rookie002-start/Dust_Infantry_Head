/**
 * @file trd_pc.cpp
 * @author Rookie002-start
 * @brief PC（自瞄）链路线程 — USB CDC ACM 单一 owner：收 PC 自瞄数据发 zbus + 发 IMU 数据给 PC
 * @version 1.0
 * @date 2026-09-23
 *
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/Pc"

#include "thread.hpp"
#include "Init_entry.hpp"
#include "pc_protocol.hpp"
#include "pc_to.hpp"
#include "imu_to.hpp"
#include "usb.hpp"

#if defined(CONFIG_DUST_TPC_FROM_BODY)
#include "from_body.hpp"        // bullet_speed 来源（下板实测弹速）
#endif
#if defined(CONFIG_TRD_BOOSTER)
#include "trd_booster.hpp"      // bullet_count 来源（累计进弹次数）
#endif

#include <stddef.h>
#include <string.h>
#include <math.h>
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

static uint32_t g_rx_frames = 0;        ///< 通过校验并发布出去的自瞄帧
static uint32_t g_rx_bad    = 0;        ///< CRC/范围检查失败的帧
static uint32_t g_tx_fail   = 0;        ///< 发给 PC 的帧被丢弃次数
static uint32_t g_last_valid_ms = 0;    ///< 最后一次收到有效自瞄帧的时刻
static uint8_t  g_last_mode = pc_proto::kRecvModeIdle;   ///< 最后一帧有效自瞄数据里的 mode
static bool     g_link_ok = false;
static uint32_t g_pub_ver  = 0;         ///< pub_pc_to 的 version 源（每发布 +1）

/// 出向数据源：IMU 快照（pull，无订阅者，version 判新）
static topic::imu_to::Message g_imu {};
static uint32_t g_imu_ver      = 0;
static uint32_t g_last_sent_ver = 0;
static uint32_t g_last_send_ms  = 0;

// ==================== 接收：组帧 + 校验 + 发布 ====================

/// 发布一帧自瞄结果（本线程是 pub_pc_to 的唯一生产者）
static void PublishAim(const PCRecvAutoAimData &d)
{
    topic::pc_to::Message msg {};
    msg.version   = ++g_pub_ver;
    msg.mode      = d.mode;
    msg.yaw_ang   = d.yaw.yaw_ang;
    msg.yaw_vel   = d.yaw.yaw_vel;
    msg.yaw_acc   = d.yaw.yaw_acc;
    msg.pitch_ang = d.pitch.pitch_ang;
    msg.pitch_vel = d.pitch.pitch_vel;
    msg.pitch_acc = d.pitch.pitch_acc;
    (void)zbus_chan_pub(&pub_pc_to, &msg, K_NO_WAIT);
}

/// 空闲帧：mode=0 + 角度 0（链路失效时发布，消费者立即失效）
static void PublishIdle()
{
    topic::pc_to::Message msg {};
    msg.version = ++g_pub_ver;
    (void)zbus_chan_pub(&pub_pc_to, &msg, K_NO_WAIT);
}

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
    if (crc_rx != pc_proto::Crc16(frame,
                                  static_cast<uint32_t>(offsetof(PCRecvAutoAimData, crc16)))) {
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

    // mode：0 空闲 / 1 自瞄不开火 / 2 自瞄开火（原样透传，语义由消费者解释）
    PublishAim(d);
    ++g_rx_frames;

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
        PublishIdle();                              // mode=0：消费者立即失效
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

    PCSendAutoAimData PC_Send_Data {};

    // 回显上板当前是否在用 PC 的自瞄数据（链路新鲜 + 最近一帧不是"空闲"）
    PC_Send_Data.mode = (g_link_ok && g_last_mode >= pc_proto::kRecvModeAim)
                            ? pc_proto::kSendModeAim
                            : pc_proto::kSendModeIdle;

    for(uint8_t i = 0; i < 4; ++i) {
        PC_Send_Data.q[i] = g_imu.quaternion[i];
    }

    PC_Send_Data.yaw.yaw_ang     = g_imu.yaw_total;   // 累计偏航（可跨圈）：世界系连续角，自瞄解算用
    PC_Send_Data.yaw.yaw_vel     = g_imu.gyro[2];     // z 轴角速度 → 偏航角速度
    PC_Send_Data.pitch.pitch_ang = g_imu.pitch;
    PC_Send_Data.pitch.pitch_vel = g_imu.gyro[1];     // y 轴角速度 → 俯仰角速度
    PC_Send_Data.bullet.bullet_speed = BulletSpeed();
    PC_Send_Data.bullet.bullet_count = BulletCount();
    PC_Send_Data.crc16 = pc_proto::get_crc16_check_sum(
        reinterpret_cast<uint8_t *>(&PC_Send_Data),
        static_cast<uint32_t>(offsetof(PCSendAutoAimData, crc16)),
        0xFFFFU);
    // printk("yaw:%f\r\n", PC_Send_Data.yaw.yaw_ang);
    // printk("pitch:%f\r\n", PC_Send_Data.pitch.pitch_ang);
    // printk("roll:%f\r\n", g_imu.roll);
    if (!usb_.Send(reinterpret_cast<const uint8_t *>(&PC_Send_Data), sizeof(PC_Send_Data))) {
        if ((++g_tx_fail % kDropLogPeriod) == 1u) {
            // LOG_WRN("usb tx drop (not configured / ep busy), total %u",
            //         static_cast<unsigned>(g_tx_fail));
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
