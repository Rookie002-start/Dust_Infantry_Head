#pragma message "Compiling Thread/Test"

#include "thread.hpp"
#include "Init_entry.hpp"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "zephyr/zbus/zbus.h"
// #include "remote_to.hpp"

#if defined(CONFIG_DEBUG_RTT)
#include <SEGGER_RTT.h>   /* SEGGER RTT，见 docs/segger_rtt.md */
#endif

LOG_MODULE_REGISTER(trd_test, LOG_LEVEL_INF);

namespace thread::test {

static Thread<2048> thread_{};

// static void ReadRemote(uint32_t &last_remote_ms)
// {
//     static topic::remote_to::Message msg{};
//     static uint32_t last_ver = 0;

//     /* remote_to 通道已无订阅者：直接轮询最新值，version 变化才算新的一帧 */
//     if (zbus_chan_read(&pub_remote_to, &msg, K_NO_WAIT) != 0 || msg.version == last_ver)
//     {
//         return;
//     }
//     last_ver = msg.version;

//     if (msg.version != 0u)
//     {
//         last_remote_ms = k_uptime_get_32();
//         /* 归一化浮点放大 100 倍按整数打印，避免依赖浮点格式化 */
//         LOG_INF("vx:%4d vy:%4d yaw:%4d pitch:%4d mode:%u",
//                 static_cast<int>(msg.chassisx * 100.0f),
//                 static_cast<int>(msg.chassisy * 100.0f),
//                 static_cast<int>(msg.yaw * 100.0f),
//                 static_cast<int>(msg.pitch * 100.0f),
//                 static_cast<unsigned>(msg.chassis_mode));
//     }
// }

static void Task(void*, void*, void*)
{
    uint32_t last_remote_ms = 0;
    for (;;)
    {
        // ReadRemote(last_remote_ms);
        printk("test");
        k_msleep(200);
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

REGISTER_INIT  (thread_init,  PreInit,   Low, "test_init");
REGISTER_THREAD(thread_start, PreThread, Low, "test_start");

}
