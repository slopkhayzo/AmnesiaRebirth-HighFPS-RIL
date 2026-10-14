#pragma once

namespace arb::limiter {

class DeadlinePacer final {
  public:
    [[nodiscard]] bool Configure(unsigned int targetFps) noexcept;
    [[nodiscard]] double NextDeadline(double nowSeconds) noexcept;
    void Reset() noexcept;

    [[nodiscard]] bool Configured() const noexcept { return configured_; }
    [[nodiscard]] double IntervalSeconds() const noexcept {
        return intervalSeconds_;
    }

  private:
    double intervalSeconds_ = 0.0;
    double deadlineSeconds_ = 0.0;
    double lastNowSeconds_ = 0.0;
    bool configured_ = false;
    bool initialized_ = false;
};

} // namespace arb::limiter
