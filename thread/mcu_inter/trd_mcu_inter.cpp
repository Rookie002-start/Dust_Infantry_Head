/**
 * @file trd_mcu_inter.cpp
 * @author qingyu
 * @brief 上板 → 下板发送线程（经典 CAN 拆帧）
 *        - 状态分片有两个来源：
 *          ① 拉取（pull）：本板已有消费者、只发一次通道的量 → RefreshFromChannels()
 *             · ImuState  ← pub_imu_to（框架 IMU 模块）
 *             · CommState ← pub_remote_to（遥控，归一化 → 指令）
 *          ② 入队（push）：只给对端用、本地没人消费的量 → PostFrame 走 k_msgq（如 AutoAimState）
 *        - 两个来源都写进同一份【本线程私有】分片缓存，按 Dir::Up 布局聚合成 62B 聚合帧
 *        - 整帧一次发出（CAN FD，FDF + BRS），周期发出
 *
 * ⚠️ 本线程是 user-can2 总线的所有者：初始化与收帧分发入口都在这里。
 *    接收侧（thread/inter_rx）只注册 CAN_RX_HANDLER，**不要**再次 Can::Init() /
 *    SetRxCallback()，否则会重复注册过滤器并覆盖分发入口。
 *
 * ⚠️ 总线带宽（CAN FD：仲裁段 1Mbps / 数据段 2Mbps，FDF + BRS）：
 *    本方向 62B 一帧 ≈ 0.34ms，对端 12B 一帧 ≈ 0.12ms，一轮合计 ≈ 0.46ms
 *    → 1ms 周期约占 46%，可行。总线忙时靠 Can::Send 的超时等待空 TX 缓冲区，不直接丢帧。
 *
 * @version 0.6
 * @date 2026-09-15
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/trd_mcu_inter"

#include "thread.hpp"
#include "Init_entry.hpp"
#include "to_mcu_tx.hpp"
#include "inter_cmd.hpp"
#include "imu_to.hpp"         // pull 来源：IMU 通道
#include "remote_to.hpp"      // pull 来源：遥控通道
#include "can.hpp"
#include "Irq_handlers.h"
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mcu_inter, LOG_LEVEL_INF);

namespace thread::mcu_inter
{
    static Thread<2048> thread_{};
    static Can mcu_inter_can{};

    static constexpr inter_cmd::Dir kDir       = inter_cmd::Dir::Up;     // 本线程发送方向
    static constexpr uint8_t        kFrameSize = 64;                     // 聚合帧缓冲上限
    static constexpr uint8_t        kStateLen  = inter_cmd::FrameLen(kDir);    // 61
    static constexpr uint32_t       kPeriodMs  = 1;                      // FD 整帧，见文件头带宽说明

    // ---- 发送等待时间 ----
    // 等空 TX 缓冲区再发，避免总线被占满时直接丢帧（一条 62B FD 帧 ≈ 0.34ms）
    static constexpr uint32_t kTxTimeoutMs = 2;

    // ---- 私有分片缓存：按契约布局表下标索引（本线程独有，不共享 → 无需锁）----
    struct Slot
    {
        uint8_t data[kFrameSize];
        uint8_t len;
        bool    valid;
    };
    static Slot slots[inter_cmd::kStateFragCount] {};

    /// 更新本方向的分片缓存
    static void UpdateSlot(inter_cmd::FrameType type, const uint8_t *data, uint8_t len)
    {
        for (uint8_t i = 0; i < inter_cmd::kStateFragCount; ++i)
        {
            const inter_cmd::StateFrag &f = inter_cmd::kStateFrags[i];
            if (f.dir != kDir || f.type != type) {
                continue;
            }
            if (len > f.len) {
                len = f.len;                       // 按契约长度截断
            }
            memcpy(slots[i].data, data, len);
            slots[i].len   = len;
            slots[i].valid = true;
            return;
        }
        LOG_WRN("not my direction/type: 0x%02x", static_cast<uint8_t>(type));
    }

    /// 按契约表把本方向分片拼成聚合帧（返回 0 = 还没有任何分片）
    static uint8_t PackState(uint8_t *buf)
    {
        bool any = false;
        memset(buf, 0, kFrameSize);

        for (uint8_t i = 0; i < inter_cmd::kStateFragCount; ++i)
        {
            if (!slots[i].valid) {
                continue;
            }
            const inter_cmd::StateFrag &f = inter_cmd::kStateFrags[i];
            if (f.dir != kDir) {
                continue;
            }
            if (static_cast<uint16_t>(f.offset) + slots[i].len > kFrameSize) {
                LOG_WRN("state slot %u overflow", i);
                continue;
            }
            memcpy(buf + f.offset, slots[i].data, slots[i].len);
            any = true;
        }
        // 定长：接收端按固定偏移解析，短帧会让尾部区域读到未定义字节
        return any ? kStateLen : 0;
    }

    /// 发一帧 CAN FD：整帧一次发完（FDF + BRS，DLC 用 can_bytes_to_dlc 编码）
    static void SendFrame(const uint8_t *buf, uint8_t len)
    {
        if (len == 0 || len > CAN_MAX_DLEN) {
            return;
        }
        can_frame tx{};
        tx.id    = inter_cmd::StateTxId(kDir);
        tx.flags = CAN_FRAME_FDF | CAN_FRAME_BRS;   // FD 帧 + 数据段 2Mbps
        tx.dlc   = can_bytes_to_dlc(len);
        memcpy(tx.data, buf, len);
        if (!mcu_inter_can.Send(&tx, K_MSEC(kTxTimeoutMs))) {
            // 等不到空 TX 缓冲区（总线被占满 / 控制器没起来）：限流记录，别刷屏
            static int64_t last_warn_ms = 0;
            const int64_t now = k_uptime_get();
            if (now - last_warn_ms >= 1000) {
                last_warn_ms = now;
                LOG_WRN("can tx busy (TX 缓冲区一直满)");
            }
        }
    }

    // ==================== ① 拉取来源（本板通道 → 分片缓存）====================

    /// 把 float 原样写进协议里的 4 字节字段（小端 IEEE754）
    static void PutFloat(uint8_t *dst, float v)
    {
        memcpy(dst, &v, sizeof(v));
    }

    // TODO(量程待定)：归一化遥控 → 目标角。下板云台会把 pitch 钳到 [kPitchMin,kPitchMax]，
    //  但 yaw_angle 是"绝对目标角"，量程不对会明显跑偏 —— 实机前必须标定这两个常数。
    static constexpr float kGimbalYawRange   = 3.1415926f;   // ±180°
    static constexpr float kGimbalPitchRange = 0.5236f;      // ±30°

    /// 数据源新鲜度阈值：超过这么久没有新样本就认为该源掉了
    /// （遥控模块自己的超时是 100ms，取同一个量级）
    static constexpr uint32_t kSourceTimeoutMs = 100;

    /// 本线程私有的"最新上板指令"：指令字段只在读到新遥控数据时更新，
    /// flags 每周期都按当前数据源状态刷新（遥控一直没连上时也要如实反映 IMU 状态）
    static inter_cmd::CommState comm_state {};

    /// 从本板通道轮询最新值组装分片（通道无订阅者，见 framework/topic/*/*.cpp）。
    ///  · version 自增 = 源还在发布；配合 kSourceTimeoutMs 判断"新鲜"
    ///  · 读失败（发布者持锁）时沿用上一轮缓存值，不清零
    ///  · 遥控掉线时模块会发一帧全 0 的复位（version 归 0），**这一帧必须采纳**：
    ///    否则上板会一直把掉线前的推杆/开关当指令发给下板（小陀螺卡住、车一直跑）
    static void RefreshFromChannels()
    {
        const int64_t now = k_uptime_get();

        // ---- ImuState ← pub_imu_to（框架 IMU 模块发布，version 自增）----
        static topic::imu_to::Message imu {};
        static uint32_t imu_ver    = 0;
        static int64_t  imu_ver_ms = 0;
        bool imu_ok = false;

        if (zbus_chan_read(&pub_imu_to, &imu, K_NO_WAIT) == 0 && imu.version != 0u) {
            if (imu.version != imu_ver) {          // 出现新样本
                imu_ver    = imu.version;
                imu_ver_ms = now;
            }
            imu_ok = (now - imu_ver_ms) <= static_cast<int64_t>(kSourceTimeoutMs);

            inter_cmd::ImuState st {};
            PutFloat(st.total_yaw_angle, imu.yaw_total);   // 累计偏航（可跨圈）
            PutFloat(st.pitch_angle,     imu.pitch);       // 俯仰角
            PutFloat(st.yaw_omega,       imu.gyro[2]);     // z 轴角速度 → 偏航角速度
            PutFloat(st.pitch_omega,     imu.gyro[1]);     // y 轴角速度 → 俯仰角速度
            UpdateSlot(inter_cmd::FrameType::StateImu,
                       reinterpret_cast<const uint8_t *>(&st), sizeof(st));
        }

        // ---- CommState ← pub_remote_to（遥控归一化量 → 上板指令）----
        static topic::remote_to::Message r {};
        static bool     remote_seen   = false;
        static uint32_t remote_ver    = 0;
        static int64_t  remote_ver_ms = 0;
        bool link_ok = false;

        // 首次必须读到有效帧（version != 0）才更新指令字段，避免把上电默认值当指令；
        // 之后连 version == 0 的"掉线复位帧"也照收（那正是失效兜底指令）。
        if (zbus_chan_read(&pub_remote_to, &r, K_NO_WAIT) == 0 &&
            (remote_seen || r.version != 0u)) {
            remote_seen = true;
            if (r.version != remote_ver) {
                remote_ver    = r.version;
                remote_ver_ms = now;
            }
            link_ok = (r.version != 0u) &&
                      ((now - remote_ver_ms) <= static_cast<int64_t>(kSourceTimeoutMs));

            comm_state.yaw_angle    = r.yaw   * kGimbalYawRange;     // 归一化 → 绝对目标角 (rad)
            comm_state.pitch_angle  = r.pitch * kGimbalPitchRange;   // 归一化 → 绝对目标角 (rad)
            comm_state.chassis_vx   = r.chassisx;                    // 下板按 [-1,1] 归一化处理
            comm_state.chassis_vy   = r.chassisy;
            comm_state.chassis_rot  = 0.0f;                          // TODO: remote_to 无自转量，来源待定
            comm_state.chassis_spin = static_cast<uint8_t>(r.chassis_mode);   // 与 SpinMode 一一对应
        }

        // 每周期都刷新标志位并写分片：遥控没连上时 CommState 就是"全 0 + 无 LinkOk"的失效指令
        // （下板据此停车），IMU 位则如实反映 IMU 新鲜度，不受遥控在不在线影响。
        comm_state.flags = static_cast<uint8_t>((link_ok ? inter_cmd::kCommFlagLinkOk : 0u) |
                                                (imu_ok  ? inter_cmd::kCommFlagImuOk  : 0u));
        UpdateSlot(inter_cmd::FrameType::StateComm,
                   reinterpret_cast<const uint8_t *>(&comm_state), sizeof(comm_state));
    }

    static void Task(void*, void*, void*)
    {
        for (;;)
        {
            const int64_t tick_start = k_uptime_get();

            // 1) 拉取通道来源的分片
            RefreshFromChannels();

            // 2) 清空队列：只给对端用的分片（push 路径）
            topic::to_mcu_tx::Message ev{};
            while (k_msgq_get(&user_can2_msgq, &ev, K_NO_WAIT) == 0)
            {
                UpdateSlot(static_cast<inter_cmd::FrameType>(ev.tag), ev.data, ev.len);
            }

            // 3) 聚合本方向状态帧 → 拆帧发出
            uint8_t buf[kFrameSize];
            const uint8_t len = PackState(buf);
            if (len > 0) {
                SendFrame(buf, len);
            }

            const int64_t elapsed = k_uptime_get() - tick_start;
            const int64_t remain  = static_cast<int64_t>(kPeriodMs) - elapsed;
            if (remain > 0) {
                k_msleep(remain);
            }
        }
    }

    bool thread_init()
    {
        const device *dev = DEVICE_DT_GET(DT_ALIAS(inter_can));
        if (!device_is_ready(dev)) {
            LOG_ERR("inter_can not ready");
            return false;
        }

        // CAN FD：必须进 FD 模式（FDOE/BRSE），否则发不出 FD 帧
        const can_filter filter{.id = 0, .mask = 0, .flags = 0};
        if (!mcu_inter_can.Init(dev, filter, CAN_MODE_FD)) {
            LOG_ERR("mcu_inter_can init fail");
            return false;
        }
        // 收帧分发入口：接收侧（thread/inter_rx）的 CAN_RX_HANDLER 依赖它
        mcu_inter_can.SetRxCallback(user_can2_rx_callback);

        // 发送结果回调：error==0 → 帧上了总线且被对端 ACK；否则 -EIO/-EBUSY/-ENETUNREACH。
        // 回调在中断上下文里跑，所以只在出错时限流打一条，正常情况完全静默。
        mcu_inter_can.SetTxCallback([](const device *, int error, void *) {
            if (error == 0) {
                return;
            }
            static int64_t last_err_ms = 0;
            const int64_t now = k_uptime_get();
            if (now - last_err_ms >= 1000) {
                last_err_ms = now;
                LOG_ERR("can tx err %d (-EIO 位/ACK 错, -EBUSY 仲裁丢失, -ENETUNREACH bus-off)",
                        error);
            }
        });

        LOG_INF("inter tx ready (dir=%s, FD frame %uB, period=%ums)",
                (kDir == inter_cmd::Dir::Up) ? "up" : "down",
                static_cast<unsigned>(kStateLen), static_cast<unsigned>(kPeriodMs));
        return true;
    }

    bool thread_start()
    {
        thread_.Start(Task, ThreadPrio::High);
        return true;
    }

    REGISTER_INIT  (thread_init,  MidInit,    High, "mcu_inter_init");
    REGISTER_THREAD(thread_start, MidThread,  High, "mcu_inter_start");
}
