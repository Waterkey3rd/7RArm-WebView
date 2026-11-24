#pragma once
#include <cstdint>

// The native preview uses the pure planner and profile, without controller ticks.
namespace os {
std::uint64_t getHardTime_uint64();
}
