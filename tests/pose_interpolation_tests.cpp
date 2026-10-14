#include "pose_interpolation.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {

void Require(const bool condition, const char* const message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

arb::clock::Sample Sample(const std::uint64_t generation,
                          const std::uint64_t older,
                          const std::uint64_t newer,
                          const double alpha) {
    arb::clock::Sample result{};
    result.kind = arb::clock::SampleKind::interpolated;
    result.generation = generation;
    result.olderSerial = older;
    result.newerSerial = newer;
    result.alpha = alpha;
    return result;
}

void TestInterpolatesEveryBone() {
    arb::pose::History history;
    arb::root::Matrix older[2]{arb::root::Identity(), arb::root::Identity()};
    arb::root::Matrix newer[2]{arb::root::Identity(), arb::root::Identity()};
    older[0].values[3] = 1.0F;
    newer[0].values[3] = 3.0F;
    older[1].values[7] = -2.0F;
    newer[1].values[7] = 2.0F;

    Require(history.Publish(10, 4, older, 2) ==
                arb::pose::PublishKind::resetGeneration,
            "first pose should seed a generation");
    Require(history.Publish(11, 4, newer, 2) ==
                arb::pose::PublishKind::appended,
            "second pose should append");
    const auto result = history.Evaluate(Sample(4, 10, 11, 0.5));
    Require(result.kind == arb::pose::SampleKind::applied,
            "two poses should interpolate");
    Require(result.pose.count == 2, "bone count should be preserved");
    Require(std::abs(result.pose.matrices[0].values[3] - 2.0F) < 1.0e-5F,
            "first bone translation should interpolate");
    Require(std::abs(result.pose.matrices[1].values[7]) < 1.0e-5F,
            "second bone translation should interpolate");
}

void TestUnchangedPoseStillAdvancesSerial() {
    arb::pose::History history;
    const arb::root::Matrix pose[1]{arb::root::Identity()};
    static_cast<void>(history.Publish(20, 8, pose, 1));
    Require(history.Publish(21, 8, pose, 1) ==
                arb::pose::PublishKind::appended,
            "unchanged bones must still receive the newer serial");
    Require(history.Evaluate(Sample(8, 20, 21, 0.25)).kind ==
                arb::pose::SampleKind::unchanged,
            "an identical pose pair should report nothing to substitute");
}

void TestStationaryBoneKeepsExactMatrix() {
    arb::pose::History history;
    arb::root::Matrix older[2]{arb::root::Identity(), arb::root::Identity()};
    older[0].values[3] = 0.1F;
    older[0].values[7] = 0.7F;
    arb::root::Matrix newer[2]{older[0], older[1]};
    newer[1].values[11] = 1.0F;

    static_cast<void>(history.Publish(30, 3, older, 2));
    static_cast<void>(history.Publish(31, 3, newer, 2));
    const auto result = history.Evaluate(Sample(3, 30, 31, 0.3));
    Require(result.kind == arb::pose::SampleKind::applied,
            "a pose with one moving bone should interpolate");
    Require(result.pose.matrices[0].values == newer[0].values,
            "a stationary bone must keep its bit-exact matrix");
    Require(std::abs(result.pose.matrices[1].values[11] - 0.3F) < 1.0e-5F,
            "the moving bone should still interpolate");
}

void TestFailClosedResets() {
    arb::pose::History history;
    arb::root::Matrix pose[1]{arb::root::Identity()};
    static_cast<void>(history.Publish(1, 1, pose, 1));
    pose[0].values[0] = std::numeric_limits<float>::quiet_NaN();
    Require(history.Publish(2, 1, pose, 1) ==
                arb::pose::PublishKind::invalid,
            "non-finite bone should invalidate history");
    Require(history.Evaluate(Sample(1, 1, 2, 0.5)).kind ==
                arb::pose::SampleKind::generationMismatch,
            "invalid pose should clear the generation");

    pose[0] = arb::root::Identity();
    static_cast<void>(history.Publish(5, 2, pose, 1));
    pose[0].values[3] = 6.0F;
    Require(history.Publish(6, 2, pose, 1) ==
                arb::pose::PublishKind::resetDiscontinuity,
            "large local translation should reseed history");
    Require(history.Evaluate(Sample(2, 5, 6, 0.5)).kind ==
                arb::pose::SampleKind::snapshotsMissing,
            "discontinuous poses must not blend");
}

void TestConsecutiveFastArticulatedPoseCanInterpolate() {
    arb::pose::History history;
    arb::root::Matrix older[1]{arb::root::Identity()};
    arb::root::Matrix newer[1]{arb::root::Identity()};
    newer[0].values[3] = 6.0F;

    static_cast<void>(history.Publish(
        40, 9, older, 1,
        arb::pose::DiscontinuityMode::allowConsecutive));
    Require(history.Publish(
                41, 9, newer, 1,
                arb::pose::DiscontinuityMode::allowConsecutive) ==
                arb::pose::PublishKind::appended,
            "consecutive fast articulated poses should append");
    const auto result = history.Evaluate(Sample(9, 40, 41, 0.5));
    Require(result.kind == arb::pose::SampleKind::applied,
            "consecutive fast articulated poses should interpolate");
    Require(std::abs(result.pose.matrices[0].values[3] - 3.0F) < 1.0e-5F,
            "fast articulated translation should interpolate");

    Require(history.Publish(
                43, 9, newer, 1,
                arb::pose::DiscontinuityMode::allowConsecutive) ==
                arb::pose::PublishKind::resetDiscontinuity,
            "a missing articulated tick must still reset history");
}

} // namespace

int main() {
    TestInterpolatesEveryBone();
    TestUnchangedPoseStillAdvancesSerial();
    TestStationaryBoneKeepsExactMatrix();
    TestFailClosedResets();
    TestConsecutiveFastArticulatedPoseCanInterpolate();
    std::cout << "pose_interpolation_tests: all checks passed\n";
    return 0;
}
