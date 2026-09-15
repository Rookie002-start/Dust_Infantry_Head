/**
 * @file to_mcu_tx.cpp
 * @author qingyu
 * @brief 上下板通信数据契约定义
 * @version 0.4
 * @date 2026-09-11
 *
 * @copyright Copyright (c) 2026
 */

#include "to_mcu_tx.hpp"

#pragma message "Compiling Topic/To_MCU_Tx"

K_MSGQ_DEFINE(user_can2_msgq, sizeof(topic::to_mcu_tx::Message), 16, 4);  // 深度 16，4 字节对齐，满时 K_NO_WAIT 丢帧
