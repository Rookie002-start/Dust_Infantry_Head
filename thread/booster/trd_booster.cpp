/**
 * @file trd_booster.cpp
 * @author Rookie002-start
 * @brief 发射机构控制线程 — 1ms 固定周期：遥控指令 → 摩擦轮升速 / 拨弹盘进弹 → 电流帧
 * @version 1.0
 * @date 2026-09-21
 *
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/Booster"

#include "thread.hpp"
#include "Init_entry.hpp"
#include "trd_booster.hpp"
#include "remote_to.hpp"
#include "pc_to.hpp"
#include "can.hpp"
#if defined(CONFIG_WDT_SUPERVISOR)
#include "health.hpp"
#endif

#include <algorithm>
#include <math.h>
#include <string.h>
#include <zephyr/drivers/gpio.h>
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

// ==================== 弹丸检测（外部中断）====================
/// 弹丸检测引脚：overlay 里 /zephyr,user 的 booster-shoot-irq-gpios（PA18，上升沿有效）。
/// 放在 /zephyr,user 是因为只有有 compatible 的节点和 /zephyr,user 才会生成属性宏。
#define BOOSTER_SHOOT_IRQ_NODE DT_PATH(zephyr_user)
static const gpio_dt_spec shoot_irq =
    GPIO_DT_SPEC_GET(BOOSTER_SHOOT_IRQ_NODE, booster_shoot_irq_gpios);
static gpio_callback shoot_irq_cb {};

/// ============================ 弹丸检测判据 ============================
/// 判据 = **密簇** + **自适应并窗**，两级：
///
///   1) 密簇：相邻沿间隔 ≤ kShootIrqClusterMs 的算一簇，一簇里沿数 ≥ kShootIrqClusterMin
///      才够格算一发 —— 真事件是一簇密集沿，干扰是零散单根沿；
///   2) 并窗：够格的簇还要和"上一次计入的那一发"间隔 ≥ 当前并窗，否则并进前一发
///      —— 一发弹的余振常在 20~50 ms 后再形成第二个密簇，只靠密度会把它算成新的一发。
///
/// 并窗是**自适应**的：并窗 = 实测射速间隔 − kShootIrqMergeSlackMs，夹在 [min, max] 内。
/// 固定并窗的两难是：射速快时窗口必须小于射速间隔（否则吃掉真弹），射速慢时窗口又要足够大
/// 才能并掉尾簇（实测尾簇最晚能到首发后约 100 ms）—— 所以按实测射速间隔动态取，
/// 连发时自动收紧、点射/慢速时自动放宽。
///
/// 调法：判"发数偏多" → 减小 kShootIrqMergeSlackMs（并窗变大）或提高 kShootIrqMergeMaxMs；
/// 判"发数偏少" → 先看簇内沿数直方图的 1/2 根两档是不是把真弹丢掉了，再决定是降
/// kShootIrqClusterMin 还是加大 kShootIrqMergeSlackMs。硬件修好后（单沿干净）设
/// kShootIrqClusterMin = 1 即可退化成最朴素的"一个沿一发"。
static constexpr uint32_t kShootIrqClusterMs  = 10;   ///< 相邻沿间隔 ≤ 本值 = 同一簇
static constexpr uint8_t  kShootIrqClusterMin = 3;    ///< 一簇至少这么多沿才算一发弹（实测 ≈9 根/发）

static constexpr uint32_t kShootIrqMergeMinMs   = 40;   ///< 并窗下限
static constexpr uint32_t kShootIrqMergeMaxMs   = 120;  ///< 并窗上限（尾簇最长约 100 ms）
static constexpr uint32_t kShootIrqMergeSlackMs = 15;   ///< 并窗与下一发首簇之间至少留的余量
static constexpr uint32_t kShootIrqPeriodInitMs = 100;  ///< 冷启动还没学到射速时假设的间隔
static constexpr uint32_t kShootIrqPeriodMinMs  = 50;   ///< 射速间隔估计的夹紧范围
static constexpr uint32_t kShootIrqPeriodMaxMs  = 1000;
static constexpr uint8_t  kShootIrqPeriodShift  = 2;    ///< 射速估计滑动平均权重 1/4（约 4 发跟上）

/// 原始上升沿数（诊断：对比 shot_count 就知道一发被算了几次）
// static atomic_t g_shoot_irq_edges  = ATOMIC_INIT(0);
/// 被并窗并掉的密簇个数 —— 同一发的余振簇（实测 11 个/102 发；突然变多说明并窗偏小）
static atomic_t g_shoot_irq_merged = ATOMIC_INIT(0);

// ---- 下面这几个只有 ISR 读写，不需要原子/加锁 ----
static uint32_t g_shoot_irq_cluster_ms  = 0;      ///< 本簇第一根沿的时刻
static uint8_t  g_shoot_irq_cluster_num = 0;      ///< 本簇已收到的沿数
static bool     g_shoot_irq_counted     = false;  ///< 本簇是否已经计过一发
static uint32_t g_shoot_irq_last_count_ms = 0;    ///< 上次"计入一发"的时刻（并窗基准）
static bool     g_shoot_irq_has_count     = false; ///< 是否已经计入过一发（第一次不做并窗）

/// 实测射速间隔（两次"计入"之间的 ms 数，滑动平均；长时间停火后的长间隔不采信）
static uint32_t g_shoot_irq_period_ms = kShootIrqPeriodInitMs;
/// 当前并窗（诊断可直接看：连发时会收紧，点射/慢速时会放宽）
// static uint32_t g_shoot_irq_merge_ms  = kShootIrqPeriodInitMs - kShootIrqMergeSlackMs;

/**
 * @brief 由实测射速间隔算当前并窗：clamp(间隔 − slack, [min, max])
 *
 * 间隔已经被夹在 [kShootIrqPeriodMinMs, kShootIrqPeriodMaxMs]，所以减法不会下溢；
 * 冷启动时估计值是 kShootIrqPeriodInitMs(100 ms) → 并窗 85 ms
 * 但仍在"射击间隔 ≥100 ms"的余量内。
 */
static uint32_t ShootIrqMergeWindow()
{
    uint32_t w = g_shoot_irq_period_ms - kShootIrqMergeSlackMs;
    if (w < kShootIrqMergeMinMs) {
        w = kShootIrqMergeMinMs;
    } else if (w > kShootIrqMergeMaxMs) {
        w = kShootIrqMergeMaxMs;
    }
    return w;
}

// // ---- 诊断：看信号结构，调判据用 ----
// /// 原始沿间隔直方图，桶 <1 / 1~5 / 5~20 / 20~50 / 50~100 / ≥100 ms
// static constexpr uint8_t kEdgeHistBucketNum = 6;
// static uint16_t g_edge_hist[kEdgeHistBucketNum] {};
// static uint32_t g_edge_prev_ms = 0;      ///< ISR 独占：上一个原始沿的时刻

// /// 簇内沿数直方图，桶 1 / 2 / 3~5 / 6~9 / ≥10 根（每根沿只属于一簇；正在进行的簇不计）
// /// 读法：前两档之和 = 被当干扰丢掉的稀疏簇个数；后三档之和 = 计入的发数 + merged。
// /// 判"发数偏少"时看前两档是不是吃掉了真弹（→ 降 kShootIrqClusterMin）；
// /// 判"发数偏多"时看后三档是不是远多于真弹造成的余振簇（→ 加大 kShootIrqMergeMs）。
// static constexpr uint8_t kClusterHistBucketNum = 5;
// static uint16_t g_cluster_hist[kClusterHistBucketNum] {};

// ==================== 周期 ====================
static constexpr uint32_t kPeriodMs = 1;
static constexpr float    kDt      = static_cast<float>(kPeriodMs) / 1000.0f;

// ==================== 机械（待实测）====================
/// 拨弹盘（M2006）减速比 36:1：模块用它把转子量折算到输出轴
static constexpr float kFeederGearRatio = 36.0f;
/// 摩擦轮（M3508 拆减速箱直驱）：模块给的 ω 就是电机侧 rad/s。
/// 若轮子仍装在 19.2:1 输出轴上，这里改 3591.f / 187.f，并按输出轴重标 kFrictionOmega
static constexpr float kFrictionGearRatio = 3591.0f / 187.0f;

/// M2006 / M3508 转矩常数（模块只用它换算 torque 字段；本线程只用 ω 与 current）
static constexpr float kFeederTorqueK   = 0.18f;
static constexpr float kFrictionTorqueK = 0.3f;

// ==================== 目标量 ====================
/// 摩擦轮目标角速度（模块单位 rad/s）；弹速 ≈ ω × 轮半径
static constexpr float kFrictionOmega      = 40.0f;
/// 到速判定（滞环）：进入 READY ≥ kFrictionReadyOn，掉出 READY < kFrictionReadyOff
static constexpr float kFrictionReadyOn  = 0.85f;
static constexpr float kFrictionReadyOff = 0.60f;
/// 升速超时：这么久还没到速判堵转/异常
static constexpr uint32_t kFrictionSpinupMs = 1500;
/// 按键去抖：两次翻转间的最小间隔
static constexpr uint32_t kFrictionKeyGapMs = 50;

/// 拨弹盘目标角速度（输出轴 rad/s）
static constexpr float kFeederOmega = 5.0f;

// 单发节拍：每 kShotIntervalMs 启动一发，单发持续进弹 kFeedPulseMs
static constexpr uint32_t kShotIntervalMs = 100;   // 射频上限 = 1000 / kShotIntervalMs
static constexpr uint32_t kFeedPulseMs    = 70;    // 每发拨弹角 ≈ kFeederOmega × kFeedPulseMs
static constexpr uint32_t kFeederRxTimeoutMs = 20; //拨弹盘电机反馈最大超时时间

// ==================== 电流限幅（保护机构与电调）====================
static constexpr float kFrictionCurrentMax = 12.0f;   // A（C620 上限 20 A）
static constexpr float kFeederCurrentMax   = 6.0f;    // A（C610 上限 10 A）

static constexpr float kFrictionOmegaKp = 0.5f;   // 摩擦轮速度环 kp
static constexpr float kFrictionOmegaKi = 0.2f;   // 摩擦轮速度环 ki
static constexpr float kFeederOmegaKp   = 0.5f;    // 拨弹盘速度环 kp
static constexpr float kFeederOmegaKi   = 0.0f;    // 拨弹盘速度环 ki

// ==================== 遥控链路 ====================
/// 遥控数据新鲜度阈值（与遥控模块自己的 100 ms 超时同量级）
static constexpr uint32_t kLinkTimeoutMs = 100;
/// PC 自瞄链路新鲜度阈值（必须与 thread/pc 的 kLinkTimeoutMs 一致）
static constexpr uint32_t kAimTimeoutMs  = 100;

// ==================== 诊断 ====================
static constexpr uint8_t  kTxPerTickMax  = 4;         // 每拍最多外发帧数
static constexpr uint32_t kDropLogPeriod = 1000;      // 丢帧/发送失败累计到该值上报一次
static constexpr uint32_t kTuneLogPeriod = 200;       // 整定日志周期（ms，LOG_DBG 级）

// ==================== 本线程私有状态（只有 Task 读写，无需加锁）====================

/// 一拍的控制意图（全部来自遥控，见文件头"控制数据语义"）
struct Command
{
    bool fire     = false;   ///< shoot_ctrl：开火
    bool reload   = false;   ///< reload_ctrl : 退弹
    bool friction = false;   ///< friction_ctrl ：开摩擦轮
    bool stop     = false;   ///< stop       ：失能状态位
    bool online   = false;   ///< 遥控链路是否有效
};

static Command g_cmd {};
static bool    g_disabled_prev   = false;   ///< 上一拍是否处于失能态（用于打沿日志）

/// 摩擦轮软开关状态机
enum class FrictionState : uint8_t
{
    Idle = 0,   ///< 关（目标 0）
    Spinup,     ///< 升速中
    Ready,      ///< 已到速，允许进弹
    Fault,      ///< 升速超时，目标 0，按一下重试
};

static FrictionState g_friction_state     = FrictionState::Idle;
static bool          g_friction_key_prev  = false;   ///< 上一拍按键状态（取按下沿）
static uint32_t      g_friction_key_ms    = 0;       ///< 上次翻转时刻（去抖）
static uint32_t      g_friction_spinup_ms = 0;       ///< 本轮升速起始时刻（超时判定）

static float g_friction_target = 0.0f;                 ///< 摩擦轮目标 ω rad/s
static float g_feeder_target   = 0.0f;                 ///< 拨弹盘目标 ω rad/s
static float g_feeder_current  = 0.0f;                 ///< 拨弹盘速度环输出电流 A（诊断用）
static float g_friction_current[kFrictionCount] {};    ///< 摩擦轮速度环输出电流 A
static bool      g_feeder_active = false;                 ///< 本拍是否真的在进弹/退弹
static atomic_t  g_feeder_rx_ms  = ATOMIC_INIT(0);        ///< 最近一帧 0x201 的时刻

static uint32_t g_drop    = 0;            ///< 队列满丢帧计数
static uint32_t g_tx_fail = 0;            ///< CAN 发送失败计数

/**
 * @brief 弹丸检测中断服务：一簇密集的上升沿（≥kShootIrqClusterMin 根）= 一发弹丸通过
 *
 * 在 GPIO 中断上下文执行（HPM GPIO 端口 ISR 里调 callback），只做 atomic 操作：
 * 不加锁、不打印、不做任何可能阻塞的动作。
 *
 * 判据（见文件上方 kShootIrq* 常量的说明）：
 *   · 先分簇（间隔 ≤kShootIrqClusterMs 算一簇），一簇沿数 ≥kShootIrqClusterMin 才够格；
 *   · 够格的簇再看并窗：距上次计入不足当前并窗（= 实测射速间隔 − 余量，自适应）就并进前一发；
 *   · 计入的同时更新射速间隔的滑动平均（停火后的长间隔不采信）。
 * 一簇只判一次（g_shoot_irq_counted），簇内后续沿直接忽略。
 */
static void ShootIrqIsr(const device *, gpio_callback *, uint32_t pins)
{
    if ((pins & BIT(shoot_irq.pin)) == 0u) {
        return;
    }

    const uint32_t now = k_uptime_get_32();

    // ---- 诊断：原始沿间隔直方图（时间差值用无符号减法，回绕也正确）----
    // const uint32_t delta = now - g_edge_prev_ms;
    // g_edge_prev_ms = now;
    // const uint8_t bucket = (delta < 1u)    ? 0
    //                      : (delta < 5u)    ? 1
    //                      : (delta < 20u)   ? 2
    //                      : (delta < 50u)   ? 3
    //                      : (delta < 100u)  ? 4
    //                                        : 5;
    // if (g_edge_hist[bucket] != UINT16_MAX) {
    //     ++g_edge_hist[bucket];
    // }

    // atomic_inc(&g_shoot_irq_edges);

    // ---- 分簇：与上一根沿间隔超过 kShootIrqClusterMs 就是新的一簇 ----
    if ((now - g_shoot_irq_cluster_ms) > kShootIrqClusterMs) {
        // 上一簇到此结束：记一笔它的沿数（诊断），然后重开一簇
        // const uint8_t n  = g_shoot_irq_cluster_num;
        // const uint8_t cb = (n <= 1u) ? 0 : (n == 2u) ? 1 : (n <= 5u) ? 2 : (n <= 9u) ? 3 : 4;
        // if (g_cluster_hist[cb] != UINT16_MAX) {
        //     ++g_cluster_hist[cb];
        // }

        g_shoot_irq_cluster_ms  = now;
        g_shoot_irq_cluster_num = 1;
        g_shoot_irq_counted     = false;
    } else {
        ++g_shoot_irq_cluster_num;
    }

    // ---- 一簇只判一次，且要等本簇沿数够格才判（够格前进来的沿不算数）----
    if (g_shoot_irq_cluster_num < kShootIrqClusterMin) {
        return;                                              // 本簇还没证明自己是真事件
    }
    if (g_shoot_irq_counted) {
        return;                                              // 本簇已经判过，簇内后续沿忽略
    }
    g_shoot_irq_counted = true;

    // 并窗基准是"上一次计入"，不是"上一根沿"：余振不会把基准往前推，下一发的首簇永远进得来。
    // 窗口用算出来的当前值（先算窗口再判决，判决之后才更新射速估计）。
    const uint32_t win = ShootIrqMergeWindow();
    if (g_shoot_irq_has_count && ((now - g_shoot_irq_last_count_ms) < win)) {
        atomic_inc(&g_shoot_irq_merged);                     // 同一发的余振密簇：并掉
        return;
    }

    // ---- 计入一发，顺便用这次间隔更新射速估计（供下一拍算并窗）----
    if (g_shoot_irq_has_count) {
        const uint32_t interval = now - g_shoot_irq_last_count_ms;
        // 停火 / 换弹后的长间隔不是射速；采信了会把并窗撑大，进而吃掉后面的真弹
        if (interval <= (g_shoot_irq_period_ms * 4u)) {
            const int32_t step = ((int32_t)interval - (int32_t)g_shoot_irq_period_ms) /
                                 (1 << kShootIrqPeriodShift);
            int32_t est = (int32_t)g_shoot_irq_period_ms + step;
            if (est < (int32_t)kShootIrqPeriodMinMs) {
                est = (int32_t)kShootIrqPeriodMinMs;
            } else if (est > (int32_t)kShootIrqPeriodMaxMs) {
                est = (int32_t)kShootIrqPeriodMaxMs;
            }
            g_shoot_irq_period_ms = (uint32_t)est;
        }
    }

    g_shoot_irq_has_count     = true;
    g_shoot_irq_last_count_ms = now;
    // g_shoot_irq_merge_ms      = ShootIrqMergeWindow();       // 诊断：当前生效的并窗
    atomic_inc(&shot_count);
}

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
 * @brief PC 自瞄开火请求：pub_pc_to 在 kAimTimeoutMs 内更新过，且 mode = 2（自瞄开火）
 */
static bool AimFireRequested(uint32_t now)
{
    static topic::pc_to::Message msg {};
    static uint32_t last_ver = 0;
    static uint32_t last_ms  = 0;

    if (zbus_chan_read(&pub_pc_to, &msg, K_NO_WAIT) == 0 && msg.version != last_ver) {
        last_ver = msg.version;
        last_ms  = now;
    }
    return (last_ms != 0u) && ((now - last_ms) <= kAimTimeoutMs) &&
           (msg.mode == topic::pc_to::kModeAimFire);
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
    // 开火：手动 shoot_ctrl，或自瞄开火（autoaim_ctrl ON + PC mode=2 且链路新鲜）
    const bool aim_fire = (msg.autoaim_ctrl == topic::remote_to::StartMode::On) &&
                          AimFireRequested(now);
    cmd.fire     = cmd.online && ((msg.shoot_ctrl == topic::remote_to::StartMode::On) ||
                                  aim_fire);
    cmd.reload   = cmd.online && (msg.reload_ctrl == topic::remote_to::StartMode::On);
    cmd.friction = cmd.online && (msg.friction_ctrl  == topic::remote_to::StartMode::On);

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
 * 摩擦轮：friction_ctrl 按下沿切换软开关，状态机 Idle/Spinup/Ready/Fault；
 *         失能 / 掉线一律回 Idle（恢复需重新手动开），非激活态目标 0（速度环主动刹停）
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

    // ---- 摩擦轮：按下沿切换的软开关 + 升速状态机 ----
    const bool disabled   = !g_cmd.online || g_cmd.stop;
    const bool pressed    = g_cmd.friction;
    const bool press_edge = pressed && !g_friction_key_prev;

    if (disabled) {
        if (g_friction_state != FrictionState::Idle) {
            g_friction_state = FrictionState::Idle;
            LOG_INF("friction OFF (disabled)");
        }
    } else if (press_edge && (now - g_friction_key_ms) >= kFrictionKeyGapMs) {
        g_friction_key_ms = now;                   // 去抖：两次翻转至少间隔 kFrictionKeyGapMs
        switch (g_friction_state) {
        case FrictionState::Fault:                  // 故障后按一下重试
            g_friction_state     = FrictionState::Spinup;
            g_friction_spinup_ms = now;
            LOG_INF("friction RETRY");
            break;
        case FrictionState::Idle:
            g_friction_state     = FrictionState::Spinup;
            g_friction_spinup_ms = now;
            LOG_INF("friction SPINUP");
            break;
        default:                                    // Spinup / Ready → 关
            g_friction_state = FrictionState::Idle;
            LOG_INF("friction OFF");
            break;
        }
    }
    g_friction_key_prev = pressed;

    // 到速判定：两个轮都要满足；READY 时用低阈值，形成滞环防抖
    bool speed_ok = true;
    const float speed_thr = (g_friction_state == FrictionState::Ready)
                                ? kFrictionReadyOff * kFrictionOmega
                                : kFrictionReadyOn * kFrictionOmega;
    for (uint8_t i = 0; i < kFrictionCount; ++i) {
        if (fabsf(friction_snap[i].omega) < speed_thr) {
            speed_ok = false;
        }
    }

    switch (g_friction_state) {
    case FrictionState::Spinup:
        if (speed_ok) {
            g_friction_state = FrictionState::Ready;
            LOG_INF("friction READY");
        } else if ((now - g_friction_spinup_ms) > kFrictionSpinupMs) {
            g_friction_state = FrictionState::Fault;
            LOG_WRN("friction FAULT (spinup timeout)");
        }
        break;
    case FrictionState::Ready:
        if (!speed_ok) {                            // 掉速 → 回升速重判
            g_friction_state     = FrictionState::Spinup;
            g_friction_spinup_ms = now;
            LOG_WRN("friction speed drop -> respin");
        }
        break;
    default:                                        // Idle / Fault 不闭环
        break;
    }

    const bool friction_enable = (g_friction_state == FrictionState::Spinup) ||
                                 (g_friction_state == FrictionState::Ready);
    g_friction_target = friction_enable ? kFrictionOmega : 0.0f;

    // ---- 进弹门控：只有 READY 才允许进弹，否则弹速不稳且容易卡弹 ----
    const bool firing_mode = g_cmd.fire && (g_friction_state == FrictionState::Ready);

    const bool reload_mode = g_cmd.reload && (g_friction_state == FrictionState::Ready);
    // ---- 拨弹盘 ----
    g_feeder_target = 0.0f;

    g_feeder_active = false;
    if (firing_mode && g_cmd.online && !g_cmd.stop) {
        g_feeder_target = -kFeederOmega;
        g_feeder_active = true;
    } else if (reload_mode && g_cmd.online && !g_cmd.stop) {
        g_feeder_target = kFeederOmega;
        g_feeder_active = true;
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

    // 反馈新鲜度：0x201 超时或从没收到过 → 当掉线（无符号减法，回绕也正确）
    const uint32_t rx_ms = (uint32_t)atomic_get(&g_feeder_rx_ms);
    const bool feeder_fb_ok = (rx_ms != 0u) &&
                              ((k_uptime_get_32() - rx_ms) <= kFeederRxTimeoutMs);

    // 不满足任一条就"放开"：输出 0 并清积分，绝不用旧 ω 去刹车
    const bool feeder_off = disable || !g_feeder_active || !feeder_fb_ok;

    if (feeder_off) {
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
                friction_omega_pid[i].Calc(i ? -g_friction_target : g_friction_target, friction_snap[i].omega);
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

    // LOG_DBG("feeder w_ref=%d w=%d i=%d | fri w_ref=%d w=%d/%d i=%d/%d | stop=%d link=%d",
    //         static_cast<int>(g_feeder_target * 100.0f),
    //         static_cast<int>(feeder_snap.omega * 100.0f),
    //         static_cast<int>(g_feeder_current * 100.0f),
    //         static_cast<int>(g_friction_target * 100.0f),
    //         static_cast<int>(friction_snap[0].omega * 100.0f),
    //         static_cast<int>(friction_snap[1].omega * 100.0f),
    //         static_cast<int>(g_friction_current[0] * 100.0f),
    //         static_cast<int>(g_friction_current[1] * 100.0f),
    //         g_cmd.stop ? 1 : 0,
    //         g_cmd.online ? 1 : 0);

    // 弹丸检测诊断：
    //   shots  = 计入的发数
    //   edges  = 原始上升沿数
    //   merged = 被并窗并掉的密簇个数（同一发的余振簇）
    //   period = 实测射速间隔（滑动平均）；merge = 当前生效的并窗
    //   edge   = 原始沿间隔直方图（<1 / 1~5 / 5~20 / 20~50 / 50~100 / ≥100 ms）
    //   cluster= 簇内沿数直方图（1 / 2 / 3~5 / 6~9 / ≥10 根）：
    //            后三档之和 ≈ 发数 + merged，前两档之和 ≈ 被丢掉的干扰簇数
    // LOG_DBG("shots=%d edges=%d merged=%d | period=%d ms merge=%d ms",
    //         static_cast<int>(atomic_get(&shot_count)),
    //         static_cast<int>(atomic_get(&g_shoot_irq_edges)),
    //         static_cast<int>(atomic_get(&g_shoot_irq_merged)),
    //         static_cast<int>(g_shoot_irq_period_ms),
    //         static_cast<int>(g_shoot_irq_merge_ms));
    // LOG_DBG("edge    %u %u %u %u %u %u",
    //         g_edge_hist[0], g_edge_hist[1], g_edge_hist[2],
    //         g_edge_hist[3], g_edge_hist[4], g_edge_hist[5]);
    // LOG_DBG("cluster %u %u %u %u %u",
    //         g_cluster_hist[0], g_cluster_hist[1], g_cluster_hist[2],
    //         g_cluster_hist[3], g_cluster_hist[4]);
}

static void Task(void*, void*, void*)
{
    for (;;)
    {
#if defined(CONFIG_WDT_SUPERVISOR)
        wdg_health::Mark(wdg_health::Service::Booster);
#endif
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
    const device *dev = DEVICE_DT_GET(DT_ALIAS(booster_can));
    if (!device_is_ready(dev)) {
        LOG_ERR("booster_can not ready");
        return false;
    }

    // DJI 电调是经典 CAN（非 FD）：用默认的 CAN_MODE_NORMAL
    const can_filter filter { .id = 0, .mask = 0, .flags = 0 };   // 全收，按 ID 分发
    if (!can3.Init(dev, filter)) {
        LOG_ERR("booster_can init fail");
        return false;
    }
    can3.SetRxCallback(user_can3_rx_callback);

    // ---- 弹丸检测：PA18 上升沿中断 → shot_count +1（与 CAN 收发无耦合）----
    if (!device_is_ready(shoot_irq.port)) {
        LOG_ERR("shoot irq gpio not ready");
        return false;
    }
    gpio_init_callback(&shoot_irq_cb, ShootIrqIsr, BIT(shoot_irq.pin));
    if (gpio_add_callback(shoot_irq.port, &shoot_irq_cb) < 0) {
        LOG_ERR("shoot irq callback add fail");
        return false;
    }
    if ((gpio_pin_configure_dt(&shoot_irq, GPIO_INPUT) < 0) ||
        (gpio_pin_interrupt_configure_dt(&shoot_irq, GPIO_INT_EDGE_TO_ACTIVE) < 0)) {
        LOG_ERR("shoot irq config fail (pin %u)", static_cast<unsigned>(shoot_irq.pin));
        return false;
    }
    LOG_INF("shoot irq ready: %s pin %u edge->active, cluster %u ms / min %u edges, "
            "merge %u~%u ms adaptive",
            shoot_irq.port->name, static_cast<unsigned>(shoot_irq.pin),
            static_cast<unsigned>(kShootIrqClusterMs),
            static_cast<unsigned>(kShootIrqClusterMin),
            static_cast<unsigned>(kShootIrqMergeMinMs),
            static_cast<unsigned>(kShootIrqMergeMaxMs));

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
        cfg.gearbox_ratio = kFrictionGearRatio; 
        cfg.torque_k      = kFrictionTorqueK;
        cfg.wheel_r       = 0.0f;                  // 摩擦轮不做线速度换算
        friction_motor[i].Init(cfg);
    }

    // ---- 速度环（输出 = 电流 A，占位增益，上车前按实车整定）----
    alg::pid::Pid::Config feeder_pid {};
    feeder_pid.kp      = kFeederOmegaKp;                  // A per rad/s
    feeder_pid.ki      = kFeederOmegaKi;
    feeder_pid.kd      = 0.0f;
    feeder_pid.iOutMax = kFeederCurrentMax;     // 积分限幅与输出限幅一致，防饱和蓄积分
    feeder_pid.outMax  = kFeederCurrentMax;     // 输出即电流，限幅保护机构
    feeder_pid.dt      = kDt;
    feeder_pid.dFirst  = alg::pid::DFirst::Enable;
    feeder_omega_pid.Init(feeder_pid);

    alg::pid::Pid::Config friction_pid {};
    friction_pid.kp      = kFrictionOmegaKp;               // A per rad/s
    friction_pid.ki      = kFrictionOmegaKi;
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
               [](uint8_t *data) {
                   atomic_set(&g_feeder_rx_ms, (atomic_val_t)k_uptime_get_32());
                   feeder_motor.CanCpltRxCallback(data);
               },
               feeder);
CAN_RX_HANDLER(BOOSTER_RX_CAN, kFrictionRxId[0],
               [](uint8_t *data) { friction_motor[0].CanCpltRxCallback(data); }, friction1);
CAN_RX_HANDLER(BOOSTER_RX_CAN, kFrictionRxId[1],
               [](uint8_t *data) { friction_motor[1].CanCpltRxCallback(data); }, friction2);

REGISTER_INIT  (thread_init,  MidInit,   Mid, "booster_init");
REGISTER_THREAD(thread_start, MidThread, Mid, "booster_start");

} // namespace thread::booster
