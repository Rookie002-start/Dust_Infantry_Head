/**
 * @file to_mcu_tx.hpp
 * @author qingyu
 * @brief 上下板通信数据契约（topic 层：只定义结构体与通道，不含业务逻辑、不含同步原语）
 * @version 0.4
 * @date 2026-09-11
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <stdint.h>
#include <zephyr/kernel.h>

extern struct k_msgq user_can2_msgq;

namespace topic::to_mcu_tx
{
    // 出向数据载体：原始载荷（方案 A）。
    // 业务线程经 inter_cmd::PostFrame() 入队（多发布者），
    // 发送线程（trd_mcu_inter）独占取出（单消费者）后组帧发出。
    struct Message
    {
        uint8_t  tag;        // 分片类型标签（= inter_cmd::FrameType，非上线 CAN ID）
        uint8_t  data[64];   // 载荷（CAN FD 最大 64 字节）
        uint8_t  len;        // 实际有效字节数
    };
}
