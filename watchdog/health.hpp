#pragma once

#include <cstdint>

namespace wdg_health {

enum class Service : uint8_t {
    Imu,
    Remote,
    Booster,
    McuInter,
    InterRx,
    Pc,
    Count,
};

void Start();
void Mark(Service service);
uint32_t FaultMask();

} // namespace wdg_health
