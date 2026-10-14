#pragma once

#include "camera_interpolation.h"
#include "presentation_clock.h"

#include <Windows.h>

#include <cstdint>
#include <string>

namespace arb::mouse_hooks {

struct Lead {
    float yaw = 0.0F;
    float pitch = 0.0F;
    bool active = false;
};

[[nodiscard]] bool Start(HMODULE executable, bool diagnosticsEnabled,
                         std::wstring& error) noexcept;
void Stop() noexcept;
[[nodiscard]] bool Enabled() noexcept;

// Called on the game thread once per completed fixed tick.
void OnTickCompleted(std::uint64_t tickSerial) noexcept;
// Drops learned state after a camera or clock discontinuity.
void ResetHistory() noexcept;
// Called once per listener-viewport render with the camera's Euler endpoints
// for the interpolated pair. Gathers mouse motion that arrived since the last
// call and returns the rotation to add to the interpolated camera.
[[nodiscard]] Lead Sample(const clock::Sample& sample,
                          const camera::Pose& older,
                          const camera::Pose& newer) noexcept;

} // namespace arb::mouse_hooks
