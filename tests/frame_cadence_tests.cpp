#include "frame_cadence.h"

#include <cmath>
#include <iostream>

namespace {

bool Near(const double left, const double right, const double epsilon) {
    return std::abs(left - right) <= epsilon;
}

} // namespace

int main() {
    arb::cadence::FrameCadence cadence;
    cadence.RecordIntervalMicroseconds(10'000);
    cadence.RecordIntervalMicroseconds(20'000);
    cadence.RecordIntervalMicroseconds(30'000);
    cadence.RecordIntervalMicroseconds(0);
    cadence.RecordIntervalMicroseconds(150'000);
    cadence.RecordIntervalMicroseconds(300'000);

    const arb::cadence::Snapshot snapshot = cadence.TakeSnapshot();
    if (snapshot.samples != 4 || snapshot.invalid != 1 ||
        snapshot.overflow != 1 || snapshot.longGaps != 1 ||
        !Near(snapshot.meanMilliseconds, 40.005, 0.02) ||
        snapshot.maxMilliseconds < 99.9) {
        std::cerr << "cadence snapshot mismatch\n";
        return 1;
    }
    const arb::cadence::Snapshot cleared = cadence.TakeSnapshot();
    if (cleared.samples != 0 || cleared.invalid != 0 || cleared.overflow != 0 ||
        cleared.longGaps != 0) {
        std::cerr << "cadence counters were not cleared\n";
        return 2;
    }
    std::cout << "frame cadence tests passed\n";
    return 0;
}

