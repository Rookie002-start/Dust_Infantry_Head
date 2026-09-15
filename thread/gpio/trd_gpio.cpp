/**
 * @file trd_gpio.cpp
 * @author qingyu
 * @brief GPIO 空转线程
 * @version 0.1
 * @date 2026-08-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma message "Compiling Thread/Gpio"

#include "thread.hpp"
#include "Init_entry.hpp"
#include "zephyr/drivers/gpio.h"
#include <zephyr/kernel.h>

namespace thread::gpio {

static Thread<2048> thread_{};

static void Task(void*, void*, void*)
{
    for (;;)
    {
        printk("hello\n");
        k_msleep(500);
    }
}

bool thread_init()
{
    return true;
}

bool thread_start()
{
    thread_.Start(Task, ThreadPrio::Low);
    return true;
}

REGISTER_INIT  (thread_init,  PreInit,   Low, "gpio_init");
REGISTER_THREAD(thread_start, PreThread, Low, "gpio_start");

} // namespace thread::gpio
