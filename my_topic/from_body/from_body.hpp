/**
 * @file from_body.hpp
 * @author qingyu
 * @brief 下板→上板接收数据契约（topic 层：只定义结构体与通道，不含业务逻辑）
 *
 * 状态通道（zbus pub_from_body）：level 语义，保留最新值，供多个消费者各自读取。
 * 下标方向命令（0x211~）当前未使用，故此处没有命令队列。
 *
 * @version 0.1
 * @date 2026-09-14
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include "inter_cmd.hpp"      // 复用共享协议结构体与布局常量

namespace topic::from_body
{
    /// 状态帧契约（zbus 通道：消费者用 zbus_chan_read 轮询最新值）
    struct Message
    {
        inter_cmd::GimbalState  gimbal{};    // 实测云台角（rad）
        inter_cmd::ShooterState shooter{};   // 子弹速度

        /// 超时判定结果。false = 下板数据已失效（上电初值为 false）
        /// ⚠️ 消费者必须先检查该标志
        bool     online{};
        /// 该帧年龄（ms）
        uint16_t age_ms{};
    };
}

ZBUS_CHAN_DECLARE(pub_from_body);
