#pragma message "Compiling Thread/Remote"

#include "remote.hpp"
#include "Init_entry.hpp"
#include "uart.hpp"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(trd_remote, LOG_LEVEL_INF);

namespace thread::remote {

static ::remote::Remote remote_ {};

/**
 * @brief 初始化 UART DMA，并将遥控器解码器配置为自动识别模式。
 */
bool thread_init()
{
    static UartDma      rx {};
    constexpr uint16_t  kTimeout = 1000;

    UartDma::Config cfg = {
        .line_cfg = {
            .baudrate    = 921600,
            .parity      = UART_CFG_PARITY_NONE,
            .stop_bits   = UART_CFG_STOP_BITS_1,
            .data_bits   = UART_CFG_DATA_BITS_8,
            .flow_ctrl   = UART_CFG_FLOW_CTRL_NONE,
        },
        .base_cfg = { .rx_timeout = kTimeout },
    };

    if (!rx.Init(DEVICE_DT_GET(DT_ALIAS(remote_uart)), cfg)) {
        LOG_ERR("uart init failed");
        return false;
    }

    remote_.Init(rx);
    return true;
}

bool thread_start()
{
    return remote_.Start(ThreadPrio::High);
}

REGISTER_INIT(thread_init,  PreInit,    High, "remote_init");
REGISTER_THREAD(thread_start, PreThread,  High, "remote_start");

} // namespace thread::remote
