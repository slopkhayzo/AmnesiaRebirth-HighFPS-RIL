#include "frame_limiter.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void Require(const bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void TestStableDeadlines() {
    arb::limiter::DeadlinePacer pacer;
    Require(pacer.Configure(120), "120 FPS was rejected");
    Require(pacer.NextDeadline(10.0) == 10.0, "first frame was delayed");
    const double second = pacer.NextDeadline(10.002);
    const double third = pacer.NextDeadline(10.009);
    Require(std::abs(second - (10.0 + 1.0 / 120.0)) < 1.0e-9,
            "second deadline was not phase locked");
    Require(std::abs(third - (10.0 + 2.0 / 120.0)) < 1.0e-9,
            "deadline accumulated render duration drift");
}

void TestBlockingPresentDoesNotDoubleLimit() {
    arb::limiter::DeadlinePacer pacer;
    Require(pacer.Configure(120), "120 FPS was rejected");
    (void)pacer.NextDeadline(20.0);
    Require(pacer.NextDeadline(20.016) == 20.016,
            "late VSync frame received an additional wait");
}

void TestDiscontinuitiesRebase() {
    arb::limiter::DeadlinePacer pacer;
    Require(!pacer.Configure(0), "zero target was accepted");
    Require(pacer.Configure(60), "60 FPS was rejected");
    (void)pacer.NextDeadline(30.0);
    Require(pacer.NextDeadline(31.0) == 31.0,
            "long stall did not reset the phase");
    Require(pacer.NextDeadline(29.0) == 29.0,
            "backward time did not reset the phase");
}

} // namespace

int main() {
    TestStableDeadlines();
    TestBlockingPresentDoesNotDoubleLimit();
    TestDiscontinuitiesRebase();
    std::cout << "frame limiter tests passed\n";
    return EXIT_SUCCESS;
}
