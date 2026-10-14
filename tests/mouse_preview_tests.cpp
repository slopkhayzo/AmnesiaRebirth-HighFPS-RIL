#include "mouse_preview.h"

#include <cmath>
#include <cstdlib>
#include <deque>
#include <iostream>

namespace {

using arb::mouse::Vec2;

void Require(const bool condition, const char* const message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

bool Near(const float left, const float right,
          const float epsilon = 1.0e-6F) {
    return std::abs(left - right) <= epsilon;
}

// Reference copy of the game's smoother: push front, drop one from the back
// if over the limit, weight by multiplier^index, normalize.
struct ReferenceSmoother {
    std::deque<Vec2> entries;
    int maximumEntries = 5;
    float multiplier = 0.7F;

    Vec2 Push(const Vec2 sample) {
        entries.push_front(sample);
        if (maximumEntries < static_cast<int>(entries.size())) {
            entries.pop_back();
        }
        float weight = 1.0F;
        float sum = 0.0F;
        Vec2 total{};
        for (const Vec2& entry : entries) {
            sum += weight;
            total.x += weight * entry.x;
            total.y += weight * entry.y;
            weight *= multiplier;
        }
        return {total.x / sum, total.y / sum};
    }

    arb::mouse::SmootherState State() const {
        arb::mouse::SmootherState state{};
        state.active = true;
        state.maximumEntries = maximumEntries;
        state.previousMultiplier = multiplier;
        state.count = entries.size();
        for (std::size_t index = 0; index < entries.size(); ++index) {
            state.entries[index] = entries[index];
        }
        return state;
    }
};

void TestUnsmoothedPredictionIsThePendingMotion() {
    arb::mouse::Carry carry{};
    Require(arb::mouse::PrepareCarry({}, carry),
            "inactive smoother was rejected");
    const Vec2 predicted =
        arb::mouse::PredictAmount(carry, {0.01F, -0.02F}, 0.4F);
    Require(Near(predicted.x, 0.01F) && Near(predicted.y, -0.02F),
            "unsmoothed prediction should equal the pending motion");
}

void TestSmoothedPredictionMeetsTheGameAtTheTick() {
    ReferenceSmoother reference;
    const Vec2 samples[]{{0.010F, 0.000F}, {0.020F, -0.004F},
                         {0.000F, 0.000F},  {-0.015F, 0.006F},
                         {0.030F, 0.001F},  {0.030F, 0.001F},
                         {0.000F, 0.000F},  {0.004F, -0.020F}};
    for (const Vec2& sample : samples) {
        arb::mouse::Carry carry{};
        Require(arb::mouse::PrepareCarry(reference.State(), carry),
                "valid smoother state was rejected");
        const Vec2 start = arb::mouse::PredictAmount(carry, {}, 0.0F);
        Require(Near(start.x, 0.0F) && Near(start.y, 0.0F),
                "prediction must start each tick interval at zero");
        const Vec2 predicted = arb::mouse::PredictAmount(carry, sample, 1.0F);
        const Vec2 actual = reference.Push(sample);
        Require(Near(predicted.x, actual.x) && Near(predicted.y, actual.y),
                "prediction at the tick must equal the game's smoothed value");
    }
}

void TestInvalidSmootherIsRejected() {
    arb::mouse::SmootherState state{};
    state.active = true;
    state.maximumEntries = 0;
    arb::mouse::Carry carry{};
    Require(!arb::mouse::PrepareCarry(state, carry),
            "a zero-length smoother was accepted");
    state.maximumEntries = 4;
    state.previousMultiplier = std::nanf("");
    Require(!arb::mouse::PrepareCarry(state, carry),
            "a non-finite multiplier was accepted");
}

void TestGainNeedsTwoAgreeingMeasurements() {
    arb::mouse::GainEstimator estimator;
    auto verified = estimator.Observe({0.02F, 0.0F}, -0.02F, 0.0F);
    Require(!estimator.HasGain() && !verified.yaw,
            "a single measurement was trusted");
    verified = estimator.Observe({0.01F, 0.0F}, -0.01F, 0.0F);
    Require(estimator.HasGain() && Near(estimator.Gain(), -1.0F, 1.0e-4F) &&
                verified.yaw && verified.pitch,
            "two agreeing ticks did not establish the gain");

    arb::mouse::GainEstimator bothAxes;
    verified = bothAxes.Observe({0.02F, -0.01F}, -0.016F, 0.008F);
    Require(bothAxes.HasGain() && Near(bothAxes.Gain(), -0.8F, 1.0e-4F) &&
                verified.yaw && verified.pitch,
            "agreeing axes in one tick did not establish the gain");
}

void TestUnexplainedRotationDisablesOnlyThatAxis() {
    arb::mouse::GainEstimator estimator;
    static_cast<void>(estimator.Observe({0.02F, -0.01F}, -0.02F, 0.01F));
    Require(estimator.HasGain(), "gain was not established");

    // A scripted yaw turn on top of ordinary mouse pitch.
    auto verified = estimator.Observe({0.0F, 0.01F}, 0.05F, -0.01F);
    Require(!verified.yaw && verified.pitch,
            "scripted yaw was attributed to the mouse");
    Require(Near(estimator.Gain(), -1.0F, 1.0e-4F),
            "an unexplained tick changed the trusted gain");

    // Pitch stopped by the camera limit while yaw keeps following.
    verified = estimator.Observe({0.01F, 0.02F}, -0.01F, 0.0F);
    Require(verified.yaw && !verified.pitch,
            "clamped pitch was treated as explained");

    // An idle mouse with a still camera is fully explained.
    verified = estimator.Observe({}, 0.0F, 0.0F);
    Require(verified.yaw && verified.pitch,
            "an idle tick was not treated as explained");
}

void TestGainFollowsSlowChangesAndRelearnsJumps() {
    arb::mouse::GainEstimator estimator;
    static_cast<void>(estimator.Observe({0.02F, 0.02F}, -0.02F, -0.02F));
    auto verified = estimator.Observe({0.02F, 0.0F}, -0.0196F, 0.0F);
    Require(verified.yaw && Near(estimator.Gain(), -0.98F, 1.0e-4F),
            "a small gain change was not followed");

    verified = estimator.Observe({0.02F, 0.0F}, -0.008F, 0.0F);
    Require(!verified.yaw, "a gain jump was accepted from one tick");
    verified = estimator.Observe({0.03F, 0.0F}, -0.012F, 0.0F);
    Require(verified.yaw && Near(estimator.Gain(), -0.4F, 1.0e-4F),
            "a repeated new gain was not adopted");
}

void TestLeadIsContinuousAcrossATick() {
    // Constant mouse speed, no smoothing, gain -1: a tick adds -0.02 yaw.
    arb::mouse::LeadInput input{};
    input.gain = -1.0F;
    input.verified = {true, true};
    input.lastAmount = {0.02F, 0.0F};

    // Just before the tick: interpolation has shown the whole last tick and
    // the pending motion is a whole tick's worth.
    input.progress = 1.0F;
    input.predictedAmount = {0.02F, 0.0F};
    const Vec2 before = arb::mouse::Lead(input);
    // Just after it: the tick delivered that motion, nothing is pending, and
    // interpolation is back at the older endpoint.
    input.progress = 0.0F;
    input.predictedAmount = {};
    const Vec2 after = arb::mouse::Lead(input);
    Require(Near(before.x, -0.02F) && Near(after.x, -0.02F),
            "lead must not jump when a tick consumes the pending motion");

    input.verified = {false, true};
    Require(Near(arb::mouse::Lead(input).x, 0.0F),
            "an unexplained axis still received a lead");
}

void TestWrapAngle() {
    constexpr float pi = 3.14159265358979323846F;
    Require(Near(arb::mouse::WrapAngle(0.25F), 0.25F), "small angle changed");
    Require(Near(arb::mouse::WrapAngle(2.0F * pi - 0.1F), -0.1F, 1.0e-5F),
            "positive wrap failed");
    Require(Near(arb::mouse::WrapAngle(-2.0F * pi + 0.1F), 0.1F, 1.0e-5F),
            "negative wrap failed");
    Require(arb::mouse::WrapAngle(std::nanf("")) == 0.0F,
            "non-finite angle was not neutralized");
}

} // namespace

int main() {
    TestUnsmoothedPredictionIsThePendingMotion();
    TestSmoothedPredictionMeetsTheGameAtTheTick();
    TestInvalidSmootherIsRejected();
    TestGainNeedsTwoAgreeingMeasurements();
    TestUnexplainedRotationDisablesOnlyThatAxis();
    TestGainFollowsSlowChangesAndRelearnsJumps();
    TestLeadIsContinuousAcrossATick();
    TestWrapAngle();
    std::cout << "mouse preview tests passed\n";
    return EXIT_SUCCESS;
}
