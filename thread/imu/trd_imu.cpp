#pragma message "Compiling Thread/Imu"

#include "imu.hpp"
#include "Init_entry.hpp"

namespace thread::imu {

static ::imu::ImuManager imu_ {};

bool thread_init()
{
    return imu_.Init(::imu::ImuStartMode::AutoCalib);
}

bool thread_start()
{
    return imu_.Start(ThreadPrio::High);
}

REGISTER_INIT  (thread_init,  EarlyInit,  Mid,  "imu_init");
REGISTER_THREAD(thread_start, LateThread, Mid,  "imu_start");

} // namespace thread::imu
