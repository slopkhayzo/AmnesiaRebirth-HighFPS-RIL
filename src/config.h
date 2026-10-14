#pragma once

#include <string>

namespace arb::config {

struct Features {
    bool renderUnlock = false;
    bool presentationLimiter = false;
    bool cameraInterpolation = false;
    bool handsRootInterpolation = false;
    bool worldRootInterpolation = false;
    bool handsPoseInterpolation = false;
    bool worldPoseInterpolation = false;
    bool ragdollPoseInterpolation = false;
    bool verletInterpolation = false;
    bool particleInterpolation = false;
    bool temporalEffectTiming = false;
    bool mouseOverlay = false;
};

struct Settings {
    bool enabled = true;
    bool diagnostics = true;
    bool telemetryEnabled = true;
    bool motionTraceEnabled = false;
    unsigned int telemetryReportIntervalSeconds = 5;
    Features features{};
    unsigned int targetFps = 0;
};

struct LoadResult {
    bool ok = false;
    Settings settings{};
    std::wstring error;
};

[[nodiscard]] LoadResult Load(const std::wstring& path);
[[nodiscard]] bool AnyBehavioralFeatureEnabled(const Features& features) noexcept;

} // namespace arb::config

