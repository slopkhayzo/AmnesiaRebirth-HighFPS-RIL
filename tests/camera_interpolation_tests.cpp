#include "camera_interpolation.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

constexpr float kPi = 3.14159265358979323846F;

void Require(const bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

arb::camera::Pose MakePose(const float x, const float pitch = 0.0F,
                           const float yaw = 0.0F,
                           const float roll = 0.0F) {
    return {{x, x * 2.0F, -x}, pitch, yaw, roll};
}

arb::clock::Sample Pair(const std::uint64_t generation,
                        const std::uint64_t older,
                        const std::uint64_t newer, const double alpha) {
    arb::clock::Sample sample{};
    sample.kind = arb::clock::SampleKind::interpolated;
    sample.generation = generation;
    sample.olderSerial = older;
    sample.newerSerial = newer;
    sample.alpha = alpha;
    return sample;
}

arb::camera::Pose MakeMatrixPose(const float x, const float yaw) {
    arb::camera::Pose pose{};
    pose.position = {x, x * 2.0F, -x};
    pose.rotateMode = arb::camera::RotateMode::matrix;
    pose.rotationMatrix = arb::root::Identity();
    pose.rotationMatrix.values[0] = std::cos(yaw);
    pose.rotationMatrix.values[2] = std::sin(yaw);
    pose.rotationMatrix.values[8] = -std::sin(yaw);
    pose.rotationMatrix.values[10] = std::cos(yaw);
    return pose;
}

void TestLinearPositionAndOrientation() {
    const auto older = MakePose(0.0F, -0.2F, 0.3F, -0.1F);
    const auto newer = MakePose(2.0F, 0.2F, 0.7F, 0.1F);
    arb::camera::Pose midpoint{};
    Require(arb::camera::Interpolate(older, newer, 0.5, midpoint),
            "valid midpoint interpolation was rejected");
    Require(std::abs(midpoint.position.x - 1.0F) < 1.0e-5F &&
                std::abs(midpoint.position.y - 2.0F) < 1.0e-5F &&
                std::abs(midpoint.position.z + 1.0F) < 1.0e-5F,
            "position was not interpolated linearly");

    arb::camera::Pose endpoint{};
    Require(arb::camera::Interpolate(older, newer, 0.0, endpoint) &&
                arb::camera::OrientationDistance(endpoint, older) < 1.0e-4F,
            "older endpoint changed");
    Require(arb::camera::Interpolate(older, newer, 1.0, endpoint) &&
                arb::camera::OrientationDistance(endpoint, newer) < 1.0e-4F,
            "newer endpoint changed");
}

void TestShortestArcAcrossYawWrap() {
    const auto older = MakePose(0.0F, 0.0F, 179.0F * kPi / 180.0F);
    const auto newer = MakePose(0.0F, 0.0F, -179.0F * kPi / 180.0F);
    arb::camera::Pose midpoint{};
    Require(arb::camera::Interpolate(older, newer, 0.5, midpoint),
            "yaw-wrap interpolation was rejected");
    Require(arb::camera::OrientationDistance(midpoint,
                                              MakePose(0.0F, 0.0F, kPi)) <
                1.0e-3F,
            "yaw wrap took the long arc");
}

void TestMatrixModeShortestArc() {
    const auto older = MakeMatrixPose(0.0F, 179.0F * kPi / 180.0F);
    const auto newer = MakeMatrixPose(2.0F, -179.0F * kPi / 180.0F);
    arb::camera::Pose midpoint{};
    Require(arb::camera::Interpolate(older, newer, 0.5, midpoint),
            "matrix camera interpolation was rejected");
    Require(midpoint.rotateMode == arb::camera::RotateMode::matrix &&
                std::abs(midpoint.position.x - 1.0F) < 1.0e-5F &&
                arb::root::OrientationDistance(
                    midpoint.rotationMatrix,
                    MakeMatrixPose(0.0F, kPi).rotationMatrix) < 1.0e-3F,
            "matrix camera did not use linear position and shortest rotation");
    Require(!arb::camera::Interpolate(older, MakePose(0.0F), 0.5, midpoint),
            "camera interpolation crossed a rotation-mode transition");
}

void TestHistoryPairingAndResets() {
    arb::camera::History history;
    Require(history.Publish(10, 4, MakePose(0.0F)).kind ==
                arb::camera::PublishKind::resetGeneration,
            "first pose did not establish generation");
    Require(history.Publish(11, 4, MakePose(1.0F)).kind ==
                arb::camera::PublishKind::appended,
            "second pose was not appended");
    const auto sampled = history.Evaluate(Pair(4, 10, 11, 0.25));
    Require(sampled.kind == arb::camera::SampleKind::applied &&
                std::abs(sampled.pose.position.x - 0.25F) < 1.0e-5F,
            "history did not use the exact serial pair and alpha");
    Require(history.Evaluate(Pair(5, 10, 11, 0.5)).kind ==
                arb::camera::SampleKind::generationMismatch,
            "history crossed a generation reset");

    Require(history.Publish(11, 4, MakePose(1.1F)).kind ==
                arb::camera::PublishKind::updatedSameTick,
            "same-tick mutation did not revise the current snapshot");
    const auto revised = history.Evaluate(Pair(4, 10, 11, 0.5));
    Require(revised.kind == arb::camera::SampleKind::applied &&
                std::abs(revised.pose.position.x - 0.55F) < 1.0e-5F,
            "same-tick revision discarded an otherwise valid history pair");
    Require(history.Publish(12, 4, MakePose(20.0F)).kind ==
                arb::camera::PublishKind::resetDiscontinuity,
            "camera teleport was interpolated");
}

void TestIdenticalOrientationHasZeroDistance() {
    // The render hook verifies an applied pose by measuring its distance
    // from the requested one, so identical inputs must never read as apart.
    float worstEuler = 0.0F;
    float worstMatrix = 0.0F;
    for (int step = 0; step < 20000; ++step) {
        const float value = static_cast<float>(step);
        const float yaw = std::fmod(value * 0.0173F, 2.0F * kPi) - kPi;
        const float pitch = std::sin(value * 0.0031F) * 1.5F;
        const float roll = std::sin(value * 0.0007F) * 0.2F;
        const auto euler = MakePose(1.0F, pitch, yaw, roll);
        worstEuler = std::max(
            worstEuler, arb::camera::OrientationDistance(euler, euler));
        const auto matrix = MakeMatrixPose(1.0F, yaw);
        worstMatrix = std::max(
            worstMatrix, arb::camera::OrientationDistance(matrix, matrix));
    }
    if (worstEuler >= 1.0e-5F || worstMatrix >= 1.0e-5F) {
        std::cerr << "worst identical-pose distance euler/matrix=" << worstEuler
                  << '/' << worstMatrix << '\n';
    }
    Require(worstEuler < 1.0e-5F,
            "identical Euler orientations measured as different");
    Require(worstMatrix < 1.0e-5F,
            "identical matrix orientations measured as different");

    const auto base = MakePose(0.0F, 0.1F, 0.4F);
    const auto turned = MakePose(0.0F, 0.1F, 0.4F + 0.01F);
    Require(std::abs(arb::camera::OrientationDistance(base, turned) - 0.01F *
                         std::cos(0.1F)) < 2.0e-4F,
            "a small real rotation was not measured accurately");
}

void TestInvalidInputFailsClosed() {
    auto invalid = MakePose(0.0F);
    invalid.yaw = std::nanf("");
    arb::camera::Pose output{};
    Require(!arb::camera::Interpolate(invalid, MakePose(1.0F), 0.5, output),
            "non-finite pose was accepted");
    Require(!arb::camera::Interpolate(MakePose(0.0F), MakePose(1.0F), 1.1,
                                      output),
            "out-of-range alpha was accepted");
}

} // namespace

int main() {
    TestLinearPositionAndOrientation();
    TestShortestArcAcrossYawWrap();
    TestMatrixModeShortestArc();
    TestHistoryPairingAndResets();
    TestIdenticalOrientationHasZeroDistance();
    TestInvalidInputFailsClosed();
    std::cout << "camera interpolation tests passed\n";
    return EXIT_SUCCESS;
}
