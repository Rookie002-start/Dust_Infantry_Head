/**
 * @file trd_booster.hpp
 * @author Rookie002-start
 * @brief 发射机构实例集合：电机对象 / 速度环 PID / CAN ID 表 / TX 队列
 * @version 1.0
 * @date 2026-09-21
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

    /// 累计已发射子弹数：弹丸检测引脚（overlay 的 /zephyr,user booster-shoot-irq-gpios，
    /// 即 PA18）。判据是"一簇密集的上升沿 = 一发"（≥kShootIrqClusterMin 根、间隔
    /// ≤kShootIrqClusterMs），零散单根沿当干扰丢掉，见 trd_booster.cpp 的 ShootIrqIsr。
    /// PC 链路线程只读，拿去填 PCSendAutoAimData.bullet_count；用 atomic 保证单调不丢。
    inline atomic_t shot_count = ATOMIC_INIT(0);
}
