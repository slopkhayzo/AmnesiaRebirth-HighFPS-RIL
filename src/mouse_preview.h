#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

// Pure presentation-side model of Rebirth's mouse-look path. The game reads
// relative mouse counts once per fixed tick, scales them, optionally smooths
// them over the last few ticks, and its player script turns the result into
// yaw/pitch. Nothing here touches the game; it only predicts what the next
// tick will add so the rendered camera can show it early.
namespace arb::mouse {

struct Vec2 {
    float x = 0.0F;
    float y = 0.0F;
};

constexpr std::size_t kMaximumSmoothingEntries = 32U;

// The game's look smoother as it stands after a completed tick.
struct SmootherState {
    bool active = false;
    // Scaled, pre-inversion samples, newest first.
    std::array<Vec2, kMaximumSmoothingEntries> entries{};
    std::size_t count = 0;
    int maximumEntries = 0;
    float previousMultiplier = 0.0F;
};

// What the smoother will contribute at the next tick besides the new sample.
struct Carry {
    Vec2 weighted{};
    float weightSum = 1.0F;
};

// Returns false when the captured smoother cannot be modelled.
[[nodiscard]] bool PrepareCarry(const SmootherState& state,
                                Carry& carry) noexcept;

// The look amount the next tick will produce if the mouse stopped now:
// `scaledPending` is the motion gathered so far and `progress` is how far
// presentation has advanced through the tick interval (0..1). At progress 1
// with the full tick's motion this equals the game's own smoothed result.
[[nodiscard]] Vec2 PredictAmount(const Carry& carry, Vec2 scaledPending,
                                 float progress) noexcept;

[[nodiscard]] float WrapAngle(float radians) noexcept;

struct Verification {
    bool yaw = false;
    bool pitch = false;
};

// Learns the script-side factor between the dispatched look amount and the
// camera's actual per-tick rotation, and reports per axis whether the last
// tick's rotation was fully explained by the mouse.
class GainEstimator final {
  public:
    void Reset() noexcept;
    [[nodiscard]] Verification Observe(Vec2 amount, float yawDelta,
                                       float pitchDelta) noexcept;
    [[nodiscard]] bool HasGain() const noexcept { return hasGain_; }
    [[nodiscard]] float Gain() const noexcept { return gain_; }

  private:
    float gain_ = 0.0F;
    float pendingCandidate_ = 0.0F;
    bool hasGain_ = false;
    bool hasPendingCandidate_ = false;
};

struct LeadInput {
    Vec2 lastAmount{};
    Vec2 predictedAmount{};
    float progress = 0.0F;
    float gain = 0.0F;
    Verification verified{};
};

// Rotation to add to the interpolated camera: the part of the last tick's
// mouse rotation that interpolation has not shown yet, plus the predicted
// rotation of the tick in progress. x is yaw, y is pitch.
[[nodiscard]] Vec2 Lead(const LeadInput& input) noexcept;

} // namespace arb::mouse
