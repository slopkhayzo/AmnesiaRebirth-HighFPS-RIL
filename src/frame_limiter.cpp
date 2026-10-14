#include "frame_limiter.h"

#include <cmath>

namespace arb::limiter {

bool DeadlinePacer::Configure(const unsigned int targetFps) noexcept {
    if (targetFps == 0U || targetFps > 1000U) {
        return false;
    }
    const double interval = 1.0 / static_cast<double>(targetFps);
    if (!configured_ || interval != intervalSeconds_) {
        intervalSeconds_ = interval;
        configured_ = true;
        Reset();
    }
    return true;
}

void DeadlinePacer::Reset() noexcept {
    initialized_ = false;
    deadlineSeconds_ = 0.0;
    lastNowSeconds_ = 0.0;
}

double DeadlinePacer::NextDeadline(const double nowSeconds) noexcept {
    if (!configured_ || !std::isfinite(nowSeconds)) {
        Reset();
        return nowSeconds;
    }
    if (!initialized_ || nowSeconds < lastNowSeconds_ ||
        nowSeconds - lastNowSeconds_ > 0.250) {
        initialized_ = true;
        deadlineSeconds_ = nowSeconds;
        lastNowSeconds_ = nowSeconds;
        return nowSeconds;
    }

    lastNowSeconds_ = nowSeconds;
    const double candidate = deadlineSeconds_ + intervalSeconds_;
    if (nowSeconds >= candidate) {
        // A blocking present (for example VSync) already paced this frame.
        // Rebase instead of adding an extra wait on top of it.
        deadlineSeconds_ = nowSeconds;
        return nowSeconds;
    }
    deadlineSeconds_ = candidate;
    return candidate;
}

} // namespace arb::limiter
