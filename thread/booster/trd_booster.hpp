/**
 * @file trd_booster.hpp
 * @author qingyu
 * @brief 发射机构实例集合：电机对象 / 速度环 PID / CAN ID 表 / TX 队列
 * @version 1.0
 * @date 2026-09-21
 *
 * 只被 thread/booster/trd_booster.cpp 使用：这里只放"实例与常量"，
 * 不放控制逻辑、不放 CAN_RX_HANDLER 注册（注册放在 .cpp，避免头文件
 * 被多处包含时在 .can_rx3 段里重复登记同一个 ID）。
 *
 * 机构构成（DJI 协议：反馈 0x20n ↔ 控制帧 0x200 第 n-1 槽）：
 *
 * | 机构 | 电机 | 电调 | 反馈 ID | 控制帧槽位 |
 * |------|------|------|---------|-----------|
 * | 拨弹盘 | M2006 | C610 | 0x201 | 0x200 第 0 槽 |
 * | 摩擦轮 A | M3508 | C620 | 0x202 | 0x200 第 1 槽 |
 * | 摩擦轮 B | M3508 | C620 | 0x203 | 0x200 第 2 槽 |
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include "Irq_handlers.h"
#include "to_can_tx.hpp"
#include "dji_c6xx.hpp"
#include "pid.hpp"
#include <cstdint>
#include <zephyr/sys/atomic.h>

/// 发射机构电机所在 bus（user-can3）；本线程是总线所有者（Init + 收帧分发）
#define BOOSTER_RX_CAN USER_RX_CAN3

namespace instance::booster
{
    /// 控制帧去向：CAN 发送 topic（本线程入队后自己外发，队列对其它生产者同样开放）
    constexpr auto *booster_tx = &user_can3_msgq;

    // ---- CAN ID：反馈 ID 决定控制帧里的槽位（反馈 0x20n ↔ 控制帧 0x200 第 n-1 槽）----
    constexpr uint16_t kCtrlTxId   = 0x200;                   ///< 三个电调共用一帧
    constexpr uint16_t kFeederRxId = 0x201;                   ///< 拨弹盘（M2006 + C610）

    constexpr uint8_t  kFrictionCount = 2;                    ///< 摩擦轮数量
    constexpr uint16_t kFrictionRxId[kFrictionCount] = {0x202, 0x203};   ///< 摩擦轮（M3508 + C620）

    constexpr uint8_t  kMotorCount = 1 + kFrictionCount;      ///< 拨弹盘 + 两枚摩擦轮

    inline motor::dji::DjiC610 feeder_motor {};               ///< 拨弹盘电机（C610，36:1）
    inline motor::dji::DjiC620 friction_motor[kFrictionCount] {};   ///< 摩擦轮电机（C620）

    inline alg::pid::Pid feeder_omega_pid {};                 ///< 拨弹盘速度环：ω → 电流
    inline alg::pid::Pid friction_omega_pid[kFrictionCount] {};  ///< 摩擦轮速度环：ω → 电流

    /// 累计进弹次数（每启动一发进弹脉冲 +1）：PC 链路拿去填 PCSendAutoAimData.bullet_count。
    /// 说明：没有弹丸检测时这是"发弹数"的近似值；跨线程只读，用 atomic 保证单调。
    inline atomic_t shot_count = ATOMIC_INIT(0);
}
