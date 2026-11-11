#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>


namespace topic::pc_to
{
    /// mode 取值（与 PC 链路契约 pc_protocol.hpp 一致）
    constexpr uint8_t kModeIdle    = 0;   // 空闲
    constexpr uint8_t kModeAim     = 1;   // 自瞄不开火
    constexpr uint8_t kModeAimFire = 2;   // 自瞄开火

    /// 状态帧契约（zbus 通道：消费者用 zbus_chan_read 轮询最新值）
    struct Message
    {
        uint32_t version = 0;   // 每次发布递增（消费者判新 / 判活）
        uint8_t mode = 0;       // 见 kMode*
        float yaw_ang;          // yaw轴角度
        float yaw_vel;          // yaw轴角速度
        float yaw_acc;          // yaw轴角加速度
        float pitch_ang;        // pitch轴角度
        float pitch_vel;        // pitch轴角速度
        float pitch_acc;        // pitch轴角加速度

#if defined(CONFIG_LATENCY_TRACE)
        uint32_t trace_rx_cycles = 0;  // 完整有效 PC 帧被确认的上板周期时刻
        uint8_t  trace_seq = 0;        // 测试序号（version 低 8 位）
        bool     trace_valid = false;  // 只有真实 PC 帧为 true，Idle 发布不采样
#endif
    };
}

ZBUS_CHAN_DECLARE(pub_pc_to);
