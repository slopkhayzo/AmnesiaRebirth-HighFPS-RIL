#include "root_interpolation.h"

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

arb::root::Matrix MakeTransform(const float x, const float yaw,
                                const float scale) {
    auto matrix = arb::root::Identity(scale);
    matrix.values[0] = std::cos(yaw) * scale;
    matrix.values[2] = std::sin(yaw) * scale;
    matrix.values[8] = -std::sin(yaw) * scale;
    matrix.values[10] = std::cos(yaw) * scale;
    matrix.values[3] = x;
    matrix.values[7] = x * 2.0F;
    matrix.values[11] = -x;
    return matrix;
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

} // namespace

int main() {
    const auto older = MakeTransform(0.0F, 179.0F * kPi / 180.0F, 0.25F);
    const auto newer = MakeTransform(2.0F, -179.0F * kPi / 180.0F, 0.25F);
    arb::root::TransformInfo info{};
    Require(arb::root::Analyze(older, info) &&
                std::abs(info.uniformScale - 0.25F) < 1.0e-5F,
            "uniform hands scale was rejected");
    arb::root::Matrix midpoint{};
    Require(arb::root::Interpolate(older, newer, 0.5, midpoint),
            "scaled root interpolation failed");
    Require(std::abs(midpoint.values[3] - 1.0F) < 1.0e-5F &&
                arb::root::OrientationDistance(
                    midpoint, MakeTransform(1.0F, kPi, 0.25F)) < 1.0e-3F,
            "scaled root did not use linear translation and shortest rotation");

    auto sheared = arb::root::Identity();
    sheared.values[1] = 0.2F;
    Require(!arb::root::Analyze(sheared, info), "shear passed validation");

    arb::root::History history;
    Require(history.Publish(10, 2, MakeTransform(0.0F, 0.0F, 1.0F)) ==
                arb::root::PublishKind::resetGeneration,
            "history generation was not established");
    Require(history.Publish(11, 2, MakeTransform(2.0F, 0.2F, 1.0F)) ==
                arb::root::PublishKind::appended,
            "second body root was not appended");
    const auto sample = history.Evaluate(Pair(2, 10, 11, 0.25));
    Require(sample.kind == arb::root::SampleKind::applied &&
                std::abs(sample.matrix.values[3] - 0.5F) < 1.0e-5F,
            "history did not evaluate its exact serial pair");

    Require(history.Publish(11, 2, MakeTransform(3.0F, 0.3F, 1.0F)) ==
                arb::root::PublishKind::resetSameTickMutation,
            "same-tick mutation was not classified as a reset");
    Require(history.Evaluate(Pair(2, 10, 11, 0.5)).kind ==
                arb::root::SampleKind::snapshotsMissing,
            "same-tick mutation left a stale interpolation pair");
    Require(history.Publish(12, 2, MakeTransform(4.0F, 0.4F, 1.0F)) ==
                arb::root::PublishKind::appended,
            "history did not recover after a same-tick mutation");

    Require(history.Publish(13, 2, MakeTransform(4.0F, 0.4F, 1.0F)) ==
                arb::root::PublishKind::appended,
            "a resting root did not receive the newer serial");
    Require(history.Evaluate(Pair(2, 12, 13, 0.5)).kind ==
                arb::root::SampleKind::unchanged,
            "a resting root pair still requested a substitution");

    std::cout << "root interpolation tests passed\n";
    return EXIT_SUCCESS;
}
