/**
 * @file trd_mcu_inter.cpp
 * @author Rookie002-start
 * @brief 上板 → 下板发送线程（CAN FD 整帧）
 *
 * @version 0.6
 * @date 2026-09-15
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/trd_mcu_inter"

#include "thread.hpp"
#include "Init_entry.hpp"
#include "inter_cmd.hpp"
#include "trd_inter_bus.hpp"  // 总线所有者（Init 在里面，本线程只当使用者）
#include "imu_to.hpp"         // pull 来源：IMU 通道
#include "remote_to.hpp"      // pull 来源：遥控通道
#include "pc_to.hpp"          // pull 来源：PC 自瞄通道
#include "can.hpp"
#include "Irq_handlers.h"
#if defined(CONFIG_WDT_SUPERVISOR)
#include "health.hpp"
#endif
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mcu_inter, LOG_LEVEL_INF);

namespace thread::mcu_inter
{
    static Thread<2048> thread_{};
    static constexpr inter_cmd::Dir kDir       = inter_cmd::Dir::Up;     // 本线程发送方向
    static constexpr uint8_t        kFrameSize = 64;                     // 聚合帧缓冲上限
    static constexpr uint8_t        kStateLen  = inter_cmd::FrameLen(kDir);    // 63
    static constexpr uint32_t       kPeriodMs  = 1;                      // FD 整帧，见文件头带宽说明

    // ---- 发送等待时间 ----
    // 等空 TX 缓冲区再发，避免总线被占满时直接丢帧（一条 63B FD 帧 ≈ 0.35ms）
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
        if (!inter_bus::Bus().Send(&tx, K_MSEC(kTxTimeoutMs))) {
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

    /// 数据源新鲜度阈值：超过这么久没有新样本就认为该源掉了。
    /// 取法是"源的 2~3 个采样周期"（判活的通用取法），两个源各自定：
    ///   · IMU：HiPNUC 100~200Hz（帧间隔 5~10ms）→ 25ms ≈ 3 帧；
    ///   · 遥控：DR16/SBUS/VT12/VT13 帧间隔 2~14ms（最慢的 SBUS 约 14ms）→ 40ms ≈ 3 帧。
    /// 之前统一取 100ms，等于允许一二十个 IMU 周期没有新样本还判"在线"：源一停，
    /// 下板会拿冻结的反馈继续闭环（环路里等于开环积分，积分一路积到限幅才被夹住）。
    static constexpr uint32_t kImuTimeoutMs    = 25;
    static constexpr uint32_t kRemoteTimeoutMs = 40;

    /// 自瞄分片（PC 链路）新鲜度阈值：必须与 thread/pc 的 kLinkTimeoutMs 一致
    static constexpr uint32_t kVisionTimeoutMs = 100;

    /// PC 偏差 → 绝对目标的符号（PC 送的是"目标角 − 当前实际角"，与下发给 PC 的
    /// imu.yaw_total / imu.pitch 同参考系，所以正常是 +1）。
    /// 若 PC 的偏差正方向与下发的角度相反，把对应项取 -1。
    static constexpr float kAimYawErrSign   = +1.0f;
    static constexpr float kAimPitchErrSign = -1.0f;

    /// 自瞄 pitch 绝对目标软限位：必须与下板 trd_gimbal.cpp 的 kPitchMin / kPitchMax 一致。
    /// 在这里先夹住，避免目标顶出界后下板软限位一直压着、要等目标自己绕回来才动。
    static constexpr float kAimPitchMin = -0.310f;
    static constexpr float kAimPitchMax =  0.350f;

    static float ClampAimPitch(float a)
    {
        return (a < kAimPitchMin) ? kAimPitchMin : ((a > kAimPitchMax) ? kAimPitchMax : a);
    }

    /// 本线程私有的"最新上板指令"：指令字段只在读到新遥控数据时更新，
    /// flags 每周期都按当前数据源状态刷新（遥控一直没连上时也要如实反映 IMU 状态）
    static inter_cmd::CommState comm_state {};

    /// 从本板通道轮询最新值组装分片（通道无订阅者，见 framework/topic/*/*.cpp）。
    ///  · version 自增 = 源还在发布；配合 kImuTimeoutMs / kRemoteTimeoutMs 判断"新鲜"
    ///  · **读失败（发布者持锁）≠ 源掉线**：只在读成功时更新缓存与 version 时间戳，
    ///    新鲜度（ok 标志）每周期都用当前时刻重算。把 ok 标志放在"读成功"分支里
    ///    是错的 —— 一次读冲突就会让 flags 丢掉 ImuOk/LinkOk，下板据此停机并走一次
    ///    "停机→解除"对齐沿（清积分 + 虚拟角重锚），现场表现就是偶发抖一下。
    ///  · 遥控掉线时模块会发一帧全 0 的复位（version 归 0），**这一帧必须采纳**：
    ///    否则上板会一直把掉线前的推杆/开关当指令发给下板（小陀螺卡住、车一直跑）
    static void RefreshFromChannels()
    {
        const int64_t now = k_uptime_get();

        // ---- ImuState ← pub_imu_to（框架 IMU 模块发布，version 自增）----
        static topic::imu_to::Message imu {};
        static uint32_t imu_ver    = 0;
        static int64_t  imu_ver_ms = 0;

        if (zbus_chan_read(&pub_imu_to, &imu, K_NO_WAIT) == 0 && imu.version != 0u) {
            if (imu.version != imu_ver) {          // 出现新样本
                imu_ver    = imu.version;
                imu_ver_ms = now;
            }

            inter_cmd::ImuState st {};
            PutFloat(st.total_yaw_angle, imu.yaw_total);   // 累计偏航（可跨圈）
            PutFloat(st.pitch_angle,     imu.pitch);       // 俯仰角
            PutFloat(st.yaw_omega,       imu.gyro[2]);     // z 轴角速度 → 偏航角速度
            PutFloat(st.pitch_omega,     imu.gyro[1]);     // y 轴角速度 → 俯仰角速度
            UpdateSlot(inter_cmd::FrameType::StateImu,
                       reinterpret_cast<const uint8_t *>(&st), sizeof(st));
        }

        // 新鲜度与"本拍读到没读到"解耦：读失败时沿用上一轮缓存，但仍然按 now 判龄。
        const bool imu_ok = (imu_ver != 0u) &&
                            ((now - imu_ver_ms) <= static_cast<int64_t>(kImuTimeoutMs));

        // ---- CommState ← pub_remote_to（遥控归一化量 → 上板指令）----
        static topic::remote_to::Message r {};
        static bool     remote_seen   = false;
        static uint32_t remote_ver    = 0;
        static int64_t  remote_ver_ms = 0;

        // 首次必须读到有效帧（version != 0）才更新指令字段，避免把上电默认值当指令；
        // 之后连 version == 0 的"掉线复位帧"也照收（那正是失效兜底指令）。
        if (zbus_chan_read(&pub_remote_to, &r, K_NO_WAIT) == 0 &&
            (remote_seen || r.version != 0u)) {
            remote_seen = true;
            if (r.version != remote_ver) {
                remote_ver    = r.version;
                remote_ver_ms = now;
            }

            comm_state.yaw_angle    = r.yaw ; 
            comm_state.pitch_angle  = r.pitch;
            comm_state.chassis_vx   = r.chassisx;                    // 下板按 [-1,1] 归一化处理
            comm_state.chassis_vy   = r.chassisy;
            comm_state.chassis_rot  = r.wheel;                          
            comm_state.chassis_spin = static_cast<uint8_t>(r.chassis_mode);   // 与 SpinMode 一一对应

            // ---- CommState.Switch：开关状态位（遥控透传，下板还要配合 CommLinkOk() 判活）----
            const bool stopped = (r.chassis_mode == topic::remote_to::ChassisMode::Stop);
            comm_state.Switch.Supercap = (r.supercap_ctrl == topic::remote_to::StartMode::On) ? 1 : 0;
            comm_state.Switch.AutoAim  = (r.autoaim_ctrl  == topic::remote_to::StartMode::On) ? 1 : 0;
            // 发射机构状态：摩擦轮开关开着、且没有踩失能位才算"已启动"
            comm_state.Switch.Booster_Status =
                ((r.shoot_ctrl == topic::remote_to::StartMode::On) && !stopped) ? 1 : 0;
            // TODO(来源待定)：遥控契约里还没有归零/快跑/刷新 UI 的按键，先固定 0
            comm_state.Switch.Gimbal_SetZero = 0;
            comm_state.Switch.Fast_Run       = 0;
            comm_state.Switch.Refresh_UI     = 0;
        }

        // 同上：读失败沿用缓存，但新鲜度按当前时刻重算（version == 0 = 掉线复位帧）
        const bool link_ok = (remote_ver != 0u) &&
                             ((now - remote_ver_ms) <= static_cast<int64_t>(kRemoteTimeoutMs));

        // ---- AutoAimState ← pub_pc_to（PC 自瞄解算）----
        // PC 每帧给的是【目标角 − 当前实际角】的**偏差**（不是绝对角、也不是逐帧增量），
        // 这里折算成**绝对角**再下发，下板按绝对角用：
        //     绝对目标 = 收到这帧时的云台实测角 + 偏差
        // 折算必须在"能看到 PC 帧边界"的地方做：本线程有 pc.version；下板拿到的只是 level 分片
        // （同一偏差值会连续出现好几拍、协议里又没序号），在下板每拍折算会让目标跟着云台跑（发散）。
        // 参考角用本板 IMU 的 imu.yaw_total / imu.pitch，与下发给 PC 的角度同参考系、也与下板
        // 位置环的反馈同参考系。角速度/角加速度是前馈量，原样透传。
        static topic::pc_to::Message pc {};
        static uint32_t pc_ver    = 0;
        static int64_t  pc_ver_ms = 0;
        static float    aim_yaw_abs   = 0.0f;   // 自瞄绝对目标（与 imu.yaw_total 同参考系）
        static float    aim_pitch_abs = 0.0f;   // 自瞄绝对目标（与 imu.pitch 同参考系）

        if (zbus_chan_read(&pub_pc_to, &pc, K_NO_WAIT) == 0 && pc.version != 0u) {
            const bool new_pc = (pc.version != pc_ver);   // 帧边界 = PC 又发了一帧
            if (new_pc) {
                pc_ver    = pc.version;
                pc_ver_ms = now;
            }

            // 失效 / 空闲 → 角度全 0（下板按 flags 忽略，不留上一帧的自瞄量）
            inter_cmd::AutoAimState st {};
            const bool fresh  = (now - pc_ver_ms) <= static_cast<int64_t>(kVisionTimeoutMs);
            const bool aim_ok = fresh && (pc.mode >= topic::pc_to::kModeAim);
            if (aim_ok) {
                // 只在来新一帧 PC 数据时折算一次；两帧之间目标保持不变（否则等于拿一个固定偏差
                // 去追云台自己，误差恒为偏差 → 一直转）。参考角差一拍（≤1ms）可忽略。
                if (new_pc) {
                    aim_yaw_abs   = imu.yaw_total + kAimYawErrSign   * pc.yaw_ang;
                    aim_pitch_abs = ClampAimPitch(imu.pitch + kAimPitchErrSign * pc.pitch_ang);
                }
                PutFloat(st.yaw_angle,   aim_yaw_abs);
                PutFloat(st.yaw_omega,   pc.yaw_vel);
                PutFloat(st.yaw_accl,    pc.yaw_acc);
                PutFloat(st.pitch_angle, aim_pitch_abs);
                PutFloat(st.pitch_omega, pc.pitch_vel);
                PutFloat(st.pitch_accl,  pc.pitch_acc);
            }
            UpdateSlot(inter_cmd::FrameType::StateAutoAim,
                       reinterpret_cast<const uint8_t *>(&st), sizeof(st));
        }

        // 标志位与分片内容分开算：读失败时沿用上一轮缓存的 pc，只有真的超过
        // kVisionTimeoutMs 没有新帧才清 VisionOk/VisionFire。
        const bool pc_fresh = (pc_ver != 0u) &&
                              ((now - pc_ver_ms) <= static_cast<int64_t>(kVisionTimeoutMs));
        const bool aim_ok_now = pc_fresh && (pc.mode >= topic::pc_to::kModeAim);
        uint8_t vision_bits = static_cast<uint8_t>(aim_ok_now ? inter_cmd::kCommFlagVisionOk : 0u);
        if (aim_ok_now && pc.mode == topic::pc_to::kModeAimFire) {
            vision_bits = static_cast<uint8_t>(vision_bits | inter_cmd::kCommFlagVisionFire);
        }

        // 只在"自瞄数据可用 ↔ 失效/空闲"的沿上打日志（1ms 周期不能刷屏）
        static bool vision_logged_ok = false;
        const bool vision_ok = (vision_bits & inter_cmd::kCommFlagVisionOk) != 0u;
        if (vision_ok != vision_logged_ok) {
            vision_logged_ok = vision_ok;
            LOG_INF("PC auto-aim %s (fire=%u)",
                    vision_ok ? "online" : "stale/idle",
                    (vision_bits & inter_cmd::kCommFlagVisionFire) ? 1u : 0u);
        }

        // 每周期都刷新标志位并写分片：遥控没连上时 CommState 就是"全 0 + 无 LinkOk"的失效指令
        // （下板据此停车），IMU 位则如实反映 IMU 新鲜度，不受遥控在不在线影响。

        comm_state.flags = static_cast<uint8_t>((link_ok ? inter_cmd::kCommFlagLinkOk : 0u) |
                                                (imu_ok  ? inter_cmd::kCommFlagImuOk  : 0u) |
                                                vision_bits);
        UpdateSlot(inter_cmd::FrameType::StateComm,
                   reinterpret_cast<const uint8_t *>(&comm_state), sizeof(comm_state));
    }

    /// 主循环：睡到"绝对截止时刻"，不要用 k_msleep(period - used)。
    /// 相对写法里的 used = k_uptime_get() - tick_start 只有 1ms 分辨率、又是按系统 tick
    /// （这里 0.5ms）量化的：循环体不足半拍就读到 0，于是每轮白送 0.5ms 以上；而且被
    /// 同优先级的兄弟线程（imu / pc / inter_rx / remote 都是 High）拖后时迟到量完全不补偿。
    /// 实测这个"1ms"广播环因此跑成了 1.57ms（见 docs/board_debug_notes.md 案例 3），
    /// 下板收到的 IMU 反馈就跟着变成 ~640Hz，环比 dt 假设多出半拍到一拍的等效延迟。
    /// 绝对截止时刻把 tick 量化和唤醒延迟一起吸收：只要循环体 < period，平均周期恒为 kPeriodMs。
    static void Task(void*, void*, void*)
    {
        const int64_t period = k_ms_to_ticks_ceil64(kPeriodMs);
        int64_t deadline = k_uptime_ticks() + period;   // 下一个周期的绝对截止 tick

        for (;;)
        {
#if defined(CONFIG_WDT_SUPERVISOR)
            wdg_health::Mark(wdg_health::Service::McuInter);
#endif
            // 1) 拉取通道来源的分片
            RefreshFromChannels();

            // 2) 聚合本方向状态帧 → 整帧发出
            uint8_t buf[kFrameSize];
            const uint8_t len = PackState(buf);
            if (len > 0) {
                SendFrame(buf, len);
            }

            // 睡到绝对截止时刻；本拍已经过点（循环体 > period）就直接重排基准，不往后欠债
            if (k_uptime_ticks() < deadline) {
                k_sleep(K_TIMEOUT_ABS_TICKS(deadline));
            } else {
                deadline = k_uptime_ticks();
            }
            deadline += period;
        }
    }

    bool thread_init()
    {
        // 发送结果回调：error==0 → 帧上了总线且被对端 ACK；否则 -EIO/-EBUSY/-ENETUNREACH。
        // 回调在中断上下文里跑，所以只在出错时限流打一条，正常情况完全静默。
        inter_bus::Bus().SetTxCallback([](const device *, int error, void *) {
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
