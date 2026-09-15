#pragma message "Compiling Thread/Test"

#include "thread.hpp"
#include "Init_entry.hpp"
#include <zephyr/kernel.h>
#include "zephyr/zbus/zbus.h"
// #include "imu_to.hpp"


namespace thread::test {

static Thread<2048> thread_{};

// static void ReadImu(uint32_t &last_imu_ms)
// {
//     static topic::imu_to::Message msg{};
//     const zbus_channel *chan = nullptr;
//     bool got = false;

//     /* 排空队列：只保留最新一帧，避免积压导致 zbus -11 刷屏 */
//     while (zbus_sub_wait(&sub_imu_to, &chan, K_NO_WAIT) == 0 && chan)
//     {
//         zbus_chan_read(chan, &msg, K_NO_WAIT);
//         got = true;
//     }

//     if (got)
//     {
//         last_imu_ms = k_uptime_get_32();
//         printk("IMU rpy:%.3f %.3f %.3f  q:%.3f %.3f %.3f %.3f  temp:%.1f\r\n",
//                (double)msg.roll, (double)msg.pitch, (double)msg.yaw,
//                (double)msg.quaternion[0], (double)msg.quaternion[1],
//                (double)msg.quaternion[2], (double)msg.quaternion[3],
//                (double)msg.temperature);
//     }
// }

static void Task(void*, void*, void*)
{
    // uint32_t tick = 0;
    // uint32_t last_imu_ms = 0;
    for (;;)
    {
        // if (++tick % 5 == 0) {
        //     printk("test\r\n");
        // }
        // ReadImu(last_imu_ms);

        // /* 每 2 秒打印一次 IMU 状态 */
        // if (tick % 20 == 0) {
        //     uint32_t now = k_uptime_get_32();
        //     if (last_imu_ms != 0 && now - last_imu_ms < 1000) {
        //         printk("[imu] OK, data flow\r\n");
        //     } else {
        //         printk("[imu] NO DATA (init failed?)\r\n");
        //     }
        // }
        printk("hello\r\n");
        k_msleep(100);
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
