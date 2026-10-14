#pragma once

#include <Windows.h>

#include <string>

namespace arb::telemetry {

[[nodiscard]] bool StartSwapRuntime(HMODULE executable, bool telemetryEnabled,
                                    unsigned int reportIntervalSeconds,
                                    bool limiterEnabled,
                                    unsigned int targetFps,
                                    std::wstring& error) noexcept;
// Performs this frame's limiter wait now instead of at the swap. Called
// just before the scene's presentation time is sampled, so every frame is
// presented a constant, minimal time after the state it shows was sampled.
void PaceBeforeRender() noexcept;
void StopSwapRuntime() noexcept;

} // namespace arb::telemetry
