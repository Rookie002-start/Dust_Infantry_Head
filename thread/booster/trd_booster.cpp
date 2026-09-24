/**
 * @file trd_booster.cpp
 * @author qingyu
 * @brief 发射机构控制线程 — 1ms 固定周期：遥控指令 → 摩擦轮升速 / 拨弹盘进弹 → 电流帧
 * @version 1.0
 * @date 2026-09-21
 *
 * ## 机构与电机
 *
 * | 机构 | 电机 | 电调 | 反馈 ID | 控制帧槽位 |
 * |------|------|------|---------|-----------|
 * | 拨弹盘 | M2006 | C610 | 0x201 | 0x200 第 0 槽 |
 * | 摩擦轮 A | M3508 | C620 | 0x202 | 0x200 第 1 槽 |
 * | 摩擦轮 B | M3508 | C620 | 0x203 | 0x200 第 2 槽 |
 *
 * 三个电调共用控制帧 0x200（DJI 协议：反馈 0x20n ↔ 控制帧第 n-1 槽），所以一拍只发一帧。
 *
 * ## 控制数据语义（topic::remote_to，本线程轮询读取）
 *
 * | 字段 | 语义 | 本线程行为 |
 * |------|------|-----------|
 * | `reload_ctrl` | **开火** | 按住进弹，按 kShotIntervalMs 限流；松开立即停 |
 * | `shoot_ctrl`  | **开摩擦轮** | 开 → 摩擦轮升速到 kFrictionOmega 并闭环稳住 |
 * | `stop`        | **失能状态位** | 置位 → 整机构失能：三台电机零电流 + 清 PID 积分 + 拨弹节拍复位 |
 *
 * `stop` 当前取遥控三态开关的停止位（见 IsStop()）；将来协议单独给出 stop 位时只改 IsStop()。
 *
 * ## 数据来源与所有权
 *
 * | 数据 | 通道 / 段 | 生产者 | 本线程的角色 |
 * |------|-----------|--------|--------------|
 * | 遥控指令 | zbus `pub_remote_to` | thread/remote | 轮询读（状态通道无订阅者） |
 * | 电机反馈 | `.can_rx3` 段 | 本线程（总线所有者） | 只注册 CAN_RX_HANDLER |
 * | 控制电流 | k_msgq `user_can3_msgq` | 本线程 | 入队后本线程外发 |
 *
 * ⚠️ 本线程是 user-can3 总线的所有者：Init 与收帧分发入口都在这里，
 *    其它线程只能注册 CAN_RX_HANDLER，不要再 Can::Init() / SetRxCallback()。
 *
 * ## 控制流
 *
 *     ReadCommand()      遥控 → fire / friction / stop + 链路判活
 *         ↓
 *     UpdateTarget()     摩擦轮目标 ω；拨弹盘射频节拍 + 到速门控 + 卡弹反转
 *         ↓
 *     ControlCalculate() 三台电机速度环 → 电流 (A)
 *         ↓
 *     FramePublish()     3 × int16 组帧（0x200）→ 队列 → FlushTx() 发到 can3
 *
 * ## 失能 / 安全策略
 *
 *   · `stop` 置位、或遥控掉线（kLinkTimeoutMs 内没有新的遥控帧）：摩擦轮停转、拨弹盘停转，
 *     清 PID 积分并强制零电流；控制帧照发（0 电流），电调不会因为超时进保护；
 *   · 摩擦轮没到速（< kFrictionReadyRatio × 目标）不允许进弹，避免弹速不稳 / 卡弹；
 *   · 摩擦轮没开（shoot_ctrl = Off）同样不允许进弹；
 *   · 拨弹盘堵转（有电流、几乎不转）超过 kJamDetectMs → 反转 kJamReverseMs 退弹后恢复；
 *   · 电流先限幅再换算成电调原始值（越界会整数回绕）。
 *
 * ## 待实测参数（占位值，实车标定后锁定）
 *
 * | 参数 | 当前值 | 说明 |
 * |------|--------|------|
 * | `kFrictionGearRatio` | 1.0 | 摩擦轮直驱（M3508 拆减速箱）；若轮子装在 19.2:1 输出轴上改 3591/187 |
 * | `kFrictionOmega` | 500 rad/s | 弹速 ≈ ω × r_轮（r = 0.03 m → ≈15 m/s） |
 * | `kFeederOmega` | 15 rad/s | 输出轴角速度（M2006 空载 ≈17.5 rad/s） |
 * | `kShotIntervalMs` / `kFeedPulseMs` | 100 / 70 ms | 射频上限 10 发/s；每发拨弹角 ≈ ω × t_脉冲 |
 * | 摩擦轮电流限幅 | 12 A | C620 上限 20 A |
 * | 拨弹盘电流限幅 | 6 A | C610 上限 10 A |
 * | 摩擦轮速度环 | kp=0.05 / ki=0.02 | 输出 = 电流 A，待整定 |
 * | 拨弹盘速度环 | kp=0.5 / ki=0.1 | 输出 = 电流 A，待整定 |
 *
 * ## 待办
 *
 *   · 反馈超时保护：DJI 模块没有反馈时间戳，ESC 掉线时速度环会顶到电流限幅，
 *     等模块补上"反馈新鲜度"后再在此处断流；
 *   · 自瞄火控（autoaim_ctrl）开火源未定：视觉开火还没有数据通道，先与手动开火共用 reload_ctrl；
 *   · 弹速闭环 / 热量限制需要弹速传感器与裁判系统数据，暂未接入。
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/Booster"

#include "thread.hpp"
#include "Init_entry.hpp"
#include "trd_booster.hpp"
#include "remote_to.hpp"
#include "can.hpp"

#include <algorithm>
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(booster, LOG_LEVEL_INF);

namespace {

/// 电调只认 ±16384（对应电调自身电流上限），换算时用它做满量程
constexpr float kRawMax = 16384.0f;

} // namespace

namespace thread::booster {

using namespace instance::booster;

static Thread<2048> thread_ {};
static Can can3 {};                             // user-can3 总线对象（本线程私有）

// ==================== 周期 ====================
static constexpr uint32_t kPeriodMs = 1;
static constexpr float    kDt      = static_cast<float>(kPeriodMs) / 1000.0f;

// ==================== 机械（待实测）====================
/// 拨弹盘（M2006）减速比 36:1：模块用它把转子量折算到输出轴
static constexpr float kFeederGearRatio = 36.0f;
/// 摩擦轮（M3508 拆减速箱直驱）：模块给的 ω 就是电机侧 rad/s。
/// 若轮子仍装在 19.2:1 输出轴上，这里改 3591.f / 187.f，并按输出轴重标 kFrictionOmega
static constexpr float kFrictionGearRatio = 1.0f;

/// M2006 / M3508 转矩常数（模块只用它换算 torque 字段；本线程只用 ω 与 current）
static constexpr float kFeederTorqueK   = 0.18f;
static constexpr float kFrictionTorqueK = 0.3f;

// ==================== 目标量 ====================
/// 摩擦轮目标角速度（模块单位 rad/s）；弹速 ≈ ω × 轮半径
static constexpr float kFrictionOmega      = 40.0f;
/// 到速判定：实测 |ω| ≥ kFrictionReadyRatio × 目标才算"摩擦轮到速"
static constexpr float kFrictionReadyRatio = 0.8f;

/// 拨弹盘目标角速度（输出轴 rad/s）
static constexpr float kFeederOmega = 5.0f;

// 单发节拍：每 kShotIntervalMs 启动一发，单发持续进弹 kFeedPulseMs
static constexpr uint32_t kShotIntervalMs = 100;   // 射频上限 = 1000 / kShotIntervalMs
static constexpr uint32_t kFeedPulseMs    = 70;    // 每发拨弹角 ≈ kFeederOmega × kFeedPulseMs

// ==================== 电流限幅（保护机构与电调）====================
static constexpr float kFrictionCurrentMax = 12.0f;   // A（C620 上限 20 A）
static constexpr float kFeederCurrentMax   = 6.0f;    // A（C610 上限 10 A）

// ==================== 卡弹自恢复（拨弹盘堵转 → 反转退弹）====================
static constexpr float    kJamCurrentThresh = 1.5f;   // 堵转判定电流 A
static constexpr float    kJamOmegaThresh   = 1.0f;   // 堵转判定转速 rad/s
static constexpr uint32_t kJamDetectMs      = 150;    // 连续堵转这么久判为卡弹
static constexpr uint32_t kJamReverseMs     = 120;    // 反转退弹时长

// ==================== 遥控链路 ====================
/// 遥控数据新鲜度阈值（与遥控模块自己的 100 ms 超时同量级）
static constexpr uint32_t kLinkTimeoutMs = 100;

// ==================== 诊断 ====================
static constexpr uint8_t  kTxPerTickMax  = 4;         // 每拍最多外发帧数
static constexpr uint32_t kDropLogPeriod = 1000;      // 丢帧/发送失败累计到该值上报一次
static constexpr uint32_t kTuneLogPeriod = 200;       // 整定日志周期（ms，LOG_DBG 级）

// ==================== 本线程私有状态（只有 Task 读写，无需加锁）====================

/// 一拍的控制意图（全部来自遥控，见文件头"控制数据语义"）
struct Command
{
    bool fire     = false;   ///< reload_ctrl：开火
    bool friction = false;   ///< shoot_ctrl ：开摩擦轮
    bool stop     = false;   ///< stop       ：失能状态位
    bool online   = false;   ///< 遥控链路是否有效
};

static Command g_cmd {};
static bool    g_disabled_prev = false;   ///< 上一拍是否处于失能态（用于打沿日志）

static float g_friction_target = 0.0f;                 ///< 摩擦轮目标 ω rad/s
static float g_feeder_target   = 0.0f;                 ///< 拨弹盘目标 ω rad/s
static float g_feeder_current  = 0.0f;                 ///< 拨弹盘速度环输出电流 A（诊断用）
static float g_friction_current[kFrictionCount] {};    ///< 摩擦轮速度环输出电流 A

static uint32_t g_next_shot_ms   = 0;     ///< 允许启动下一发的时刻（射频限流）
static uint32_t g_shot_end_ms    = 0;     ///< 本发进弹结束时刻
static uint32_t g_stall_ms       = 0;     ///< 堵转起始时刻（0 = 未堵转）
static uint32_t g_reverse_end_ms = 0;     ///< 反转退弹结束时刻

static uint32_t g_drop    = 0;            ///< 队列满丢帧计数
static uint32_t g_tx_fail = 0;            ///< CAN 发送失败计数

/**
 * @brief 失能状态位（stop）
 *
 * 遥控协议里"停止 / 失能"由三态开关的停止位表达（ChassisMode::Stop）。
 * 失能是安全动作：只要遥控在线就无条件采信，不受开火 / 开关门控影响。
 * 将来协议单独给出 stop 位时，只改这一处即可。
 */
static bool IsStop(const topic::remote_to::Message &msg)
{
    return msg.chassis_mode == topic::remote_to::ChassisMode::Stop;
}

/**
 * @brief 读遥控指令并判活
 *
 * 状态通道 pub_remote_to 无订阅者，这里轮询最新值：
 *   · version 变化 = 有新遥控帧 → 刷新时间戳；
 *   · 超过 kLinkTimeoutMs 没有新帧（遥控掉线时模块只发一帧全 0 复位帧）→ 判为失效；
 *   · 读忙（发布者正在更新）时沿用上一轮指令，但判活时间照常走。
 */
static void ReadCommand()
{
    static topic::remote_to::Message msg {};
    static uint32_t last_ver = 0;
    static uint32_t last_ms  = 0;

    const uint32_t now = k_uptime_get_32();

    if (zbus_chan_read(&pub_remote_to, &msg, K_NO_WAIT) == 0 && msg.version != last_ver) {
        last_ver = msg.version;
        last_ms  = now;
    }

    Command cmd {};
    cmd.online   = (last_ms != 0u) && ((now - last_ms) <= kLinkTimeoutMs);
    cmd.stop     = IsStop(msg);
    cmd.fire     = cmd.online && (msg.reload_ctrl == topic::remote_to::StartMode::On);
    cmd.friction = cmd.online && (msg.shoot_ctrl  == topic::remote_to::StartMode::On);

    // 失能 / 恢复的沿只打一条日志（1ms 周期里不能刷屏）
    const bool disabled = cmd.stop || !cmd.online;
    if (disabled != g_disabled_prev) {
        g_disabled_prev = disabled;
        if (!disabled) {
            LOG_INF("booster ready (remote link ok)");
        } else if (cmd.stop) {
            LOG_INF("booster DISABLED (stop bit) -> all current zero");
        } else {
            LOG_INF("booster DISABLED (remote link lost) -> all current zero");
        }
    }

    g_cmd = cmd;
}

/**
 * @brief 目标量生成：摩擦轮升速 + 拨弹盘进弹节拍（含卡弹反转）
 *
 * 摩擦轮：开 → kFrictionOmega；其余（未开 / 失能 / 掉线）→ 0（速度环主动刹停）
 * 拨弹盘：只有 开火 && 未失能 && 链路有效 && 摩擦轮到速 且 距上一发 ≥ kShotIntervalMs
 *         才启动一发，单发持续 kFeedPulseMs；禁止进弹时目标立即归零。
 *
 * @param feeder_snap   本拍拨弹盘快照（卡弹判定用，与速度环同一帧）
 * @param friction_snap 本拍摩擦轮快照（与速度环用同一帧，避免跨帧读数）
 */
static void UpdateTarget(const motor::dji::DjiC610::Snapshot &feeder_snap,
                         const motor::dji::DjiC620::Snapshot *friction_snap)
{
    const uint32_t now = k_uptime_get_32();

    // ---- 摩擦轮 ----
    const bool friction_on = g_cmd.online && g_cmd.friction && !g_cmd.stop;
    g_friction_target = friction_on ? kFrictionOmega : 0.0f;

    // ---- 到速门控：两轮都到速才允许进弹，否则弹速不稳且容易卡弹 ----
    bool ready = friction_on;
    for (uint8_t i = 0; i < kFrictionCount; ++i) {
        if (fabsf(friction_snap[i].omega) < kFrictionReadyRatio * kFrictionOmega) {
            ready = false;
        }
    }

    // ---- 卡弹自恢复：进弹模式下有电流却转不动 → 攒够 kJamDetectMs 就反转退弹 ----
    // 堵转计时跨过单发之间的空档（空档里电流会掉下去，但不代表机构通了），
    // 只要中途转起来过就清零；一停止进弹（松开 / 失能 / 退弹中）也清零。
    const bool firing_mode = g_cmd.fire && ready && (now >= g_reverse_end_ms);

    if (!firing_mode) {
        g_stall_ms = 0u;
    } else if (fabsf(feeder_snap.omega) >= kJamOmegaThresh) {
        g_stall_ms = 0u;                                 // 转起来了：没卡
    } else if (fabsf(feeder_snap.current) > kJamCurrentThresh) {
        if (g_stall_ms == 0u) {
            g_stall_ms = now;
        } else if ((now - g_stall_ms) >= kJamDetectMs) {
            g_reverse_end_ms = now + kJamReverseMs;      // 反转退弹
            g_shot_end_ms    = now;                      // 打断本发，交给反转
            g_stall_ms       = 0u;
            LOG_WRN("feeder jam -> reverse %u ms", static_cast<unsigned>(kJamReverseMs));
        }
    }

    // ---- 拨弹盘 ----
    g_feeder_target = 0.0f;

    // 1) 反转退弹优先级最高（此时不给进弹）
    if (now < g_reverse_end_ms) {
        g_feeder_target = -kFeederOmega;
        return;
    }

    // 2) 禁止进弹（松开 / 失能 / 掉线 / 摩擦轮没到速）：清节拍，下次按下立即响应
    if (!firing_mode) {
        g_shot_end_ms  = now;
        g_next_shot_ms = now;
        return;
    }

    // 3) 射频限流后启动一发
    if (now >= g_next_shot_ms) {
        g_shot_end_ms  = now + kFeedPulseMs;
        g_next_shot_ms = now + kShotIntervalMs;
        atomic_inc(&shot_count);                     // 累计进弹次数（PC 链路读它填 bullet_count）
    }

    // 4) 只在进弹脉冲内给转速
    if (now < g_shot_end_ms) {
        g_feeder_target = kFeederOmega;
    }
}

/**
 * @brief 三台电机的速度环：目标 ω → 电流 (A)
 *
 * 电调吃的是电流，所以速度环输出直接就是电流，输出限幅即电流保护；
 * 失能 / 掉线时清积分并强制零电流（不拿陈旧状态闭环）。
 */
static void ControlCalculate(const motor::dji::DjiC610::Snapshot &feeder_snap,
                             const motor::dji::DjiC620::Snapshot *friction_snap)
{
    const bool disable = g_cmd.stop || !g_cmd.online;

    if (disable) {
        feeder_omega_pid.SetIntegralError(0.0f);
        g_feeder_current = 0.0f;
    } else {
        g_feeder_current = feeder_omega_pid.Calc(g_feeder_target, feeder_snap.omega);
    }

    for (uint8_t i = 0; i < kFrictionCount; ++i)
    {
        if (disable) {
            friction_omega_pid[i].SetIntegralError(0.0f);
            g_friction_current[i] = 0.0f;
        } else {
            g_friction_current[i] =
                friction_omega_pid[i].Calc(g_friction_target, friction_snap[i].omega);
        }
    }
}

/**
 * @brief 组帧 → 投入 CAN 发送队列（消费者：本线程的 FlushTx）
 *
 * DJI 协议：一帧 0x200 带 4 个 int16 电流，高字节在前；第 n 槽对应反馈 0x20n。
 * 越界会整数回绕，所以一律先按电调电流上限限幅再换算。
 */
static void FramePublish()
{
    topic::to_can_tx::Message msg {};
    msg.tx_id = kCtrlTxId;

    auto pack = [&msg](uint8_t slot, float amp, float amp_max) {
        const float    clamped = std::clamp(amp, -amp_max, amp_max);
        const uint16_t raw =
            static_cast<uint16_t>(static_cast<int16_t>(clamped * (kRawMax / amp_max)));
        msg.data[slot * 2 + 0] = static_cast<uint8_t>(raw >> 8);
        msg.data[slot * 2 + 1] = static_cast<uint8_t>(raw & 0xFF);
    };

    pack(0, g_feeder_current, kFeederCurrentMax);
    for (uint8_t i = 0; i < kFrictionCount; ++i) {
        pack(static_cast<uint8_t>(1 + i), g_friction_current[i], kFrictionCurrentMax);
    }

    if (k_msgq_put(booster_tx, &msg, K_NO_WAIT) != 0) {
        // 发送线程来不及取帧（正常不会发生）：丢当前帧并计数，控制环不阻塞
        if ((++g_drop % kDropLogPeriod) == 1u) {
            LOG_WRN("can tx queue full: %u frames dropped", static_cast<unsigned>(g_drop));
        }
    }
}

/**
 * @brief 外发本线程排在 user_can3_msgq 里的帧
 *
 * 本线程是 can3 总线所有者；队列同时对外开放，其它线程可以把帧投进来。
 */
static void FlushTx()
{
    topic::to_can_tx::Message msg {};
    uint8_t sent = 0;

    while (sent < kTxPerTickMax && k_msgq_get(booster_tx, &msg, K_NO_WAIT) == 0)
    {
        // 契约固定 8 字节载荷（不能用 sizeof(tx.data)：CAN FD 模式下是 64 字节）
        can_frame tx {};
        tx.id  = msg.tx_id;
        tx.dlc = 8;
        memcpy(tx.data, msg.data, sizeof(msg.data));

        if (!can3.Send(&tx)) {
            if ((++g_tx_fail % kDropLogPeriod) == 1u) {
                LOG_WRN("can3 send fail id=0x%02x (total %u)",
                        static_cast<unsigned>(msg.tx_id), static_cast<unsigned>(g_tx_fail));
            }
        }
        ++sent;
    }
}

/**
 * @brief 发射机构控制主循环（固定 1ms 周期）
 *
 * 周期用 tick 计时：1ms = 2 个 tick（CONFIG_SYS_CLOCK_TICKS_PER_SEC=2000），
 * 用 k_uptime_get() 的 ms 粒度算余量会把周期抖成 1~2ms。
 */
/**
 * @brief 整定用日志（LOG_DBG 级，默认被 LOG_LEVEL_INF 挡掉）
 *
 * 整定时把本文件开头的 LOG_MODULE_REGISTER 级别改成 LOG_LEVEL_DBG 即可在 RTT 上看到
 * 目标 / 实测 / 电流；Zephyr log 默认不支持 %f，所以这里的量都放大成整数打印。
 */
static void LogTuning(const motor::dji::DjiC610::Snapshot &feeder_snap,
                      const motor::dji::DjiC620::Snapshot *friction_snap)
{
    static uint32_t last_ms = 0;

    const uint32_t now = k_uptime_get_32();
    if ((now - last_ms) < kTuneLogPeriod) {
        return;
    }
    last_ms = now;

    LOG_DBG("feeder w_ref=%d w=%d i=%d | fri w_ref=%d w=%d/%d i=%d/%d | stop=%d link=%d",
            static_cast<int>(g_feeder_target * 100.0f),
            static_cast<int>(feeder_snap.omega * 100.0f),
            static_cast<int>(g_feeder_current * 100.0f),
            static_cast<int>(g_friction_target * 100.0f),
            static_cast<int>(friction_snap[0].omega * 100.0f),
            static_cast<int>(friction_snap[1].omega * 100.0f),
            static_cast<int>(g_friction_current[0] * 100.0f),
            static_cast<int>(g_friction_current[1] * 100.0f),
            g_cmd.stop ? 1 : 0,
            g_cmd.online ? 1 : 0);
}

static void Task(void*, void*, void*)
{
    for (;;)
    {
        const int64_t tick_start = k_uptime_ticks();

        ReadCommand();

        // 每台电机一拍一次快照：转速与电流必须来自同一帧
        const auto feeder_snap = feeder_motor.ReadAll();
        motor::dji::DjiC620::Snapshot friction_snap[kFrictionCount] {};
        for (uint8_t i = 0; i < kFrictionCount; ++i) {
            friction_snap[i] = friction_motor[i].ReadAll();
        }

        UpdateTarget(feeder_snap, friction_snap);
        ControlCalculate(feeder_snap, friction_snap);
        FramePublish();
        FlushTx();
        LogTuning(feeder_snap, friction_snap);

        const int64_t period = k_ms_to_ticks_ceil64(kPeriodMs);
        const int64_t used   = k_uptime_ticks() - tick_start;
        if (used < period) {
            k_sleep(K_TICKS(period - used));
        }
    }
}

bool thread_init()
{
    // ---- user-can3：本线程是总线所有者（Init + 收帧分发入口）----
    const device *dev = DEVICE_DT_GET(DT_ALIAS(user_can3));
    if (!device_is_ready(dev)) {
        LOG_ERR("user_can3 not ready");
        return false;
    }

    // DJI 电调是经典 CAN（非 FD）：用默认的 CAN_MODE_NORMAL
    const can_filter filter { .id = 0, .mask = 0, .flags = 0 };   // 全收，按 ID 分发
    if (!can3.Init(dev, filter)) {
        LOG_ERR("user_can3 init fail");
        return false;
    }
    can3.SetRxCallback(user_can3_rx_callback);

    // ---- 电机：拨弹盘（M2006 + C610）+ 摩擦轮 ×2（M3508 + C620）----
    motor::dji::DjiC610::Config feeder_cfg {};
    feeder_cfg.rx_id         = kFeederRxId;
    feeder_cfg.gearbox_ratio = kFeederGearRatio;   // 36:1 → 模块给的 ω 是输出轴角速度
    feeder_cfg.torque_k      = kFeederTorqueK;
    feeder_cfg.wheel_r       = 0.0f;               // 拨弹盘不做线速度换算
    feeder_motor.Init(feeder_cfg);

    for (uint8_t i = 0; i < kFrictionCount; ++i)
    {
        motor::dji::DjiC620::Config cfg {};
        cfg.rx_id         = kFrictionRxId[i];
        cfg.gearbox_ratio = kFrictionGearRatio;    // 1.0 → 模块给的 ω 是电机侧角速度
        cfg.torque_k      = kFrictionTorqueK;
        cfg.wheel_r       = 0.0f;                  // 摩擦轮不做线速度换算
        friction_motor[i].Init(cfg);
    }

    // ---- 速度环（输出 = 电流 A，占位增益，上车前按实车整定）----
    alg::pid::Pid::Config feeder_pid {};
    feeder_pid.kp      = 0.5f;                  // A per rad/s
    feeder_pid.ki      = 0.1f;
    feeder_pid.kd      = 0.0f;
    feeder_pid.iOutMax = kFeederCurrentMax;     // 积分限幅与输出限幅一致，防饱和蓄积分
    feeder_pid.outMax  = kFeederCurrentMax;     // 输出即电流，限幅保护机构
    feeder_pid.dt      = kDt;
    feeder_pid.dFirst  = alg::pid::DFirst::Enable;
    feeder_omega_pid.Init(feeder_pid);

    alg::pid::Pid::Config friction_pid {};
    friction_pid.kp      = 0.05f;               // A per rad/s
    friction_pid.ki      = 0.02f;
    friction_pid.kd      = 0.0f;
    friction_pid.iOutMax = kFrictionCurrentMax;
    friction_pid.outMax  = kFrictionCurrentMax;
    friction_pid.dt      = kDt;
    friction_pid.dFirst  = alg::pid::DFirst::Enable;

    for (auto &pid : friction_omega_pid) {
        pid.Init(friction_pid);
    }

    // 日志里不打印浮点：Zephyr log 的 cbprintf 默认不带 %f 支持
    LOG_INF("booster ready: feeder 0x%03x, friction 0x%03x/0x%03x, tx 0x%03x, %u ms",
            static_cast<unsigned>(kFeederRxId),
            static_cast<unsigned>(kFrictionRxId[0]),
            static_cast<unsigned>(kFrictionRxId[kFrictionCount - 1]),
            static_cast<unsigned>(kCtrlTxId),
            static_cast<unsigned>(kPeriodMs));
    return true;
}

bool thread_start()
{
    thread_.Start(Task, ThreadPrio::High);
    return true;
}

// CAN 收帧注册：每个反馈 ID 一个入口（分发器只给帧数据，不给 ID）
CAN_RX_HANDLER(BOOSTER_RX_CAN, kFeederRxId,
               [](uint8_t *data) { feeder_motor.CanCpltRxCallback(data); }, feeder);
CAN_RX_HANDLER(BOOSTER_RX_CAN, kFrictionRxId[0],
               [](uint8_t *data) { friction_motor[0].CanCpltRxCallback(data); }, friction1);
CAN_RX_HANDLER(BOOSTER_RX_CAN, kFrictionRxId[1],
               [](uint8_t *data) { friction_motor[1].CanCpltRxCallback(data); }, friction2);

REGISTER_INIT  (thread_init,  MidInit,   Mid, "booster_init");
REGISTER_THREAD(thread_start, MidThread, Mid, "booster_start");

} // namespace thread::booster
