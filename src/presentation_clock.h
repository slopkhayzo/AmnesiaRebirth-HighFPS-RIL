#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace arb::clock {

enum class ResetReason : std::uint32_t {
    none,
    reconfigured,
    explicitDiscontinuity,
    nonMonotonicWallTime,
    nonMonotonicTick,
    missingSnapshots,
    longStall,
};

enum class SampleKind : std::uint32_t {
    unavailable,
    interpolated,
    reset,
};

struct PublishResult {
    bool accepted = false;
    ResetReason resetReason = ResetReason::none;
};

struct Sample {
    SampleKind kind = SampleKind::unavailable;
    ResetReason resetReason = ResetReason::none;
    std::uint64_t generation = 0;
    std::uint64_t olderSerial = 0;
    std::uint64_t newerSerial = 0;
    double alpha = 0.0;
    double targetTime = 0.0;
};

class PresentationClock final {
  public:
    static constexpr std::size_t kHistoryCapacity = 2;

    [[nodiscard]] bool Configure(double nativeStepSeconds,
                                 double longStallSeconds = 0.250) noexcept;
    [[nodiscard]] PublishResult Publish(std::uint64_t serial, double wallTime,
                                        bool discontinuity = false) noexcept;
    [[nodiscard]] Sample Evaluate(double wallTime, double interpolationAlpha,
                                  bool phaseValid = true) noexcept;
    void Reset(ResetReason reason = ResetReason::explicitDiscontinuity) noexcept;

    [[nodiscard]] bool Configured() const noexcept { return configured_; }
    [[nodiscard]] double NativeStepSeconds() const noexcept {
        return nativeStepSeconds_;
    }
    [[nodiscard]] std::size_t SnapshotCount() const noexcept { return count_; }
    [[nodiscard]] std::uint64_t Generation() const noexcept { return generation_; }

  private:
    struct SnapshotState {
        std::uint64_t serial = 0;
        double wallArrivalTime = 0.0;
    };

    void Clear(ResetReason reason) noexcept;
    void Append(std::uint64_t serial, double wallTime) noexcept;
    [[nodiscard]] const SnapshotState& At(std::size_t logicalIndex) const noexcept;

    std::array<SnapshotState, kHistoryCapacity> snapshots_{};
    std::size_t first_ = 0;
    std::size_t count_ = 0;
    std::uint64_t generation_ = 0;
    double nativeStepSeconds_ = 0.0;
    double longStallSeconds_ = 0.250;
    double lastSampleWallTime_ = 0.0;
    bool configured_ = false;
    bool hasLastSample_ = false;
};

static_assert(PresentationClock::kHistoryCapacity == 2U,
              "Interpolation must retain only previous/current ticks.");

[[nodiscard]] bool LogicTimerAlpha(double nextTickMilliseconds,
                                   double stepMilliseconds,
                                   double speedMultiplier,
                                   double nowMilliseconds,
                                   double presentationDelayTicks,
                                   double& alpha,
                                   double* rawAlpha = nullptr) noexcept;
[[nodiscard]] const wchar_t* ResetReasonName(ResetReason reason) noexcept;
[[nodiscard]] const wchar_t* SampleKindName(SampleKind kind) noexcept;

} // namespace arb::clock

