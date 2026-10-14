#include "presentation_clock.h"

#include <array>
#include <cmath>
#include <iostream>

namespace {

bool Near(const double left, const double right,
          const double epsilon = 1.0e-9) {
    return std::abs(left - right) <= epsilon;
}

bool TestRate(const double rate) {
    arb::clock::PresentationClock clock;
    const double step = 1.0 / rate;
    if (!clock.Configure(step) || !clock.Configured() ||
        !Near(clock.NativeStepSeconds(), step)) {
        return false;
    }
    if (!clock.Publish(10, 1.0).accepted ||
        !clock.Publish(11, 1.0 + step).accepted) {
        return false;
    }
    const arb::clock::Sample sample = clock.Evaluate(1.0 + step, 0.25);
    return sample.kind == arb::clock::SampleKind::interpolated &&
           sample.olderSerial == 10 && sample.newerSerial == 11 &&
           Near(sample.alpha, 0.25) &&
           Near(sample.targetTime, (10.0 + 0.25) * step);
}

} // namespace

int main() {
    for (const double rate : std::array{50.0, 60.0, 120.0, 144.0}) {
        if (!TestRate(rate)) {
            std::cerr << "parameterized rate failed: " << rate << '\n';
            return 1;
        }
    }

    arb::clock::PresentationClock clock;
    if (clock.Configure(0.0) || clock.Configure(2.0) ||
        !clock.Configure(1.0 / 60.0)) {
        std::cerr << "configuration validation failed\n";
        return 2;
    }
    static_cast<void>(clock.Publish(1, 1.0));
    const auto missing = clock.Publish(3, 1.02);
    if (!missing.accepted ||
        missing.resetReason != arb::clock::ResetReason::missingSnapshots ||
        clock.SnapshotCount() != 1) {
        std::cerr << "missing snapshot reset failed\n";
        return 3;
    }

    double alpha = 0.0;
    double raw = 0.0;
    if (!arb::clock::LogicTimerAlpha(1016.0, 16.0, 1.0, 1008.0, 0.0,
                                    alpha, &raw) ||
        !Near(raw, 0.5) || !Near(alpha, 0.5)) {
        std::cerr << "logic timer alpha failed\n";
        return 4;
    }

    std::cout << "presentation clock tests passed\n";
    return 0;
}

