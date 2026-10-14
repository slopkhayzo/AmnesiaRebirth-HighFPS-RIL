#pragma once

#include "camera_attachment.h"
#include "presentation_clock.h"

#include <Windows.h>

#include <cstdint>
#include <string>

namespace arb::root_probe {

[[nodiscard]] bool Start(HMODULE executable, bool rootInterpolationEnabled,
                         bool poseInterpolationEnabled,
                         bool worldInterpolationEnabled,
                         bool ragdollPoseInterpolationEnabled,
                         bool diagnosticsEnabled,
                         std::wstring& error) noexcept;
void Commit(std::uint64_t tickSerial,
            std::uint64_t clockGeneration) noexcept;
void ResetHistory() noexcept;
// Extra yaw, about a vertical axis through the given point, applied to the
// player's hands, body, and held roots for the next render scope only.
void SetViewYawLead(float yawRadians, float pivotX, float pivotY,
                    float pivotZ) noexcept;
// If the camera is rigidly attached to a PlayerBody bone across the sampled
// tick pair, replaces `presented` with the camera pose on the interpolated
// skeleton and returns true. `presented.rotation` must hold the camera's
// independently interpolated rotation matrix on entry.
[[nodiscard]] bool SolveAttachedCamera(const clock::Sample& sample,
                                       const attach::CameraEndpoint& older,
                                       const attach::CameraEndpoint& newer,
                                       attach::CameraEndpoint& presented)
    noexcept;
[[nodiscard]] bool BeginRender(const clock::Sample& sample) noexcept;
void EndRender() noexcept;
void Stop() noexcept;

} // namespace arb::root_probe
