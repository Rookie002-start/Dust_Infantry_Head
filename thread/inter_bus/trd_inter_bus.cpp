/**
 * @file trd_inter_bus.cpp
 * @author qingyu
 * @brief 上下板通信总线所有者实现（无 REGISTER_THREAD，只有 REGISTER_INIT）
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 */

#pragma message "Compiling Thread/Inter_Bus"

#include "trd_inter_bus.hpp"
#include "Init_entry.hpp"
#include "Irq_handlers.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(inter_bus, LOG_LEVEL_INF);

namespace thread::inter_bus
{
    static Can bus{};

    Can& Bus()
    {
        return bus;
    }

    /// 只做三件事：拿设备 → Init（进 FD 模式 + 注册过滤器 + can_start）→ 设收帧分发入口。
    /// 阶段 MidInit 保证早于所有 MidThread（发送/接收线程）启动。
    static bool thread_init()
    {
        const device *dev = DEVICE_DT_GET(DT_ALIAS(inter_can));
        if (!device_is_ready(dev)) {
            LOG_ERR("inter_can not ready");
            return false;
        }

        const can_filter filter{.id = 0, .mask = 0, .flags = 0};
        if (!bus.Init(dev, filter, CAN_MODE_FD)) {
            LOG_ERR("inter bus init fail");
            return false;
        }

        // 收帧分发入口：接收线程（thread/inter_rx）的 CAN_RX_HANDLER 依赖它
        bus.SetRxCallback(user_can2_rx_callback);
        LOG_INF("inter bus ready (%s, FD)", dev->name);
        return true;
    }

    REGISTER_INIT(thread_init, MidInit, High, "inter_bus_init");

} // namespace thread::inter_bus
