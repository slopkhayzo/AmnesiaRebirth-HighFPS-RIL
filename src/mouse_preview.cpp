#include "mouse_preview.h"

#include <algorithm>
#include <cmath>

namespace arb::mouse {
namespace {

constexpr float kPi = 3.14159265358979323846F;
// Below this look amount a single tick is too small to measure a gain from.
constexpr float kMinimumCandidateAmount = 2.0e-4F;
constexpr float kRelativeTolerance = 0.03F;
constexpr float kAbsoluteTolerance = 1.5e-4F;
constexpr float kMinimumGain = 0.02F;
constexpr float kMaximumGain = 20.0F;

bool Consistent(const float left, const float right) noexcept {
    return std::abs(left - right) <=
           kRelativeTolerance * std::max(std::abs(left), std::abs(right));
}

bool Candidate(const float amount, const float delta, float& gain) noexcept {
    if (std::abs(amount) < kMinimumCandidateAmount) {
        return false;
    }
    gain = delta / amount;
    return std::isfinite(gain) && std::abs(gain) >= kMinimumGain &&
           std::abs(gain) <= kMaximumGain;
}

bool Explained(const float gain, const float amount,
               const float delta) noexcept {
    const float expected = gain * amount;
    return std::abs(delta - expected) <=
           kAbsoluteTolerance + kRelativeTolerance * std::abs(expected);
}

} // namespace

bool PrepareCarry(const SmootherState& state, Carry& carry) noexcept {
    carry = {};
    if (!state.active) {
        return true;
    }
    if (state.maximumEntries < 1 ||
        state.count > kMaximumSmoothingEntries ||
        !std::isfinite(state.previousMultiplier) ||
        state.previousMultiplier < 0.0F || state.previousMultiplier > 4.0F) {
        return false;
    }
    // The game pushes the new sample to the front and then drops one entry
    // from the back only if the list grew past its limit.
    const auto limit = static_cast<std::size_t>(state.maximumEntries);
    const std::size_t nextCount =
        limit < state.count + 1U ? state.count : state.count + 1U;
    float weight = 1.0F;
    float weightSum = nextCount == 0U ? 0.0F : 1.0F;
    for (std::size_t index = 1; index < nextCount; ++index) {
        weight *= state.previousMultiplier;
        const Vec2& entry = state.entries[index - 1U];
        if (!std::isfinite(entry.x) || !std::isfinite(entry.y)) {
            return false;
        }
        carry.weighted.x += weight * entry.x;
        carry.weighted.y += weight * entry.y;
        weightSum += weight;
    }
    if (!(weightSum > 0.0F) || !std::isfinite(weightSum) ||
        !std::isfinite(carry.weighted.x) || !std::isfinite(carry.weighted.y)) {
        return false;
    }
    carry.weightSum = weightSum;
    return true;
}

Vec2 PredictAmount(const Carry& carry, const Vec2 scaledPending,
                   const float progress) noexcept {
    const float clamped = std::clamp(progress, 0.0F, 1.0F);
    return {(scaledPending.x + clamped * carry.weighted.x) / carry.weightSum,
            (scaledPending.y + clamped * carry.weighted.y) / carry.weightSum};
}

float WrapAngle(const float radians) noexcept {
    if (!std::isfinite(radians)) {
        return 0.0F;
    }
    float wrapped = std::fmod(radians + kPi, 2.0F * kPi);
    if (wrapped < 0.0F) {
        wrapped += 2.0F * kPi;
    }
    return wrapped - kPi;
}

void GainEstimator::Reset() noexcept {
    gain_ = 0.0F;
    pendingCandidate_ = 0.0F;
    hasGain_ = false;
    hasPendingCandidate_ = false;
}

Verification GainEstimator::Observe(const Vec2 amount, const float yawDelta,
                                    const float pitchDelta) noexcept {
    float yawCandidate = 0.0F;
    float pitchCandidate = 0.0F;
    const bool hasYaw = Candidate(amount.x, yawDelta, yawCandidate);
    const bool hasPitch = Candidate(amount.y, pitchDelta, pitchCandidate);

    if (hasGain_) {
        const Verification verified{Explained(gain_, amount.x, yawDelta),
                                    Explained(gain_, amount.y, pitchDelta)};
        // Follow slow, legitimate changes such as a faded look-speed factor.
        if (verified.yaw && hasYaw) {
            gain_ = yawCandidate;
        } else if (verified.pitch && hasPitch && !hasYaw) {
            gain_ = pitchCandidate;
        }
        if ((hasYaw && verified.yaw) || (hasPitch && verified.pitch) ||
            (!hasYaw && !hasPitch)) {
            hasPendingCandidate_ = false;
            return verified;
        }
    }

    // No trusted gain explains this tick. Adopt a new one only when two
    // independent measurements agree: both axes now, or this tick and the
    // previous unexplained one.
    bool adopted = false;
    if (hasYaw && hasPitch && Consistent(yawCandidate, pitchCandidate)) {
        gain_ = 0.5F * (yawCandidate + pitchCandidate);
        adopted = true;
    } else if (hasYaw || hasPitch) {
        const float candidate = hasYaw ? yawCandidate : pitchCandidate;
        if (hasPendingCandidate_ && Consistent(candidate, pendingCandidate_)) {
            gain_ = candidate;
            adopted = true;
        } else {
            pendingCandidate_ = candidate;
            hasPendingCandidate_ = true;
        }
    }
    if (adopted) {
        hasGain_ = true;
        hasPendingCandidate_ = false;
    }
    if (!hasGain_) {
        return {};
    }
    return {Explained(gain_, amount.x, yawDelta),
            Explained(gain_, amount.y, pitchDelta)};
}

Vec2 Lead(const LeadInput& input) noexcept {
    const float remaining = 1.0F - std::clamp(input.progress, 0.0F, 1.0F);
    Vec2 lead{};
    if (input.verified.yaw) {
        lead.x = input.gain * (remaining * input.lastAmount.x +
                               input.predictedAmount.x);
    }
    if (input.verified.pitch) {
        lead.y = input.gain * (remaining * input.lastAmount.y +
                               input.predictedAmount.y);
    }
    if (!std::isfinite(lead.x) || !std::isfinite(lead.y)) {
        return {};
    }
    return lead;
}

} // namespace arb::mouse
