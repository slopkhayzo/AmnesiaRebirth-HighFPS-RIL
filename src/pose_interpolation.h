#pragma once

#include "presentation_clock.h"
#include "root_interpolation.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace arb::pose {

constexpr std::size_t kMaximumBones = 256U;

struct Pose {
    std::array<root::Matrix, kMaximumBones> matrices{};
    std::size_t count = 0;
};

enum class PublishKind : std::uint32_t {
    appended,
    invalid,
    resetGeneration,
    resetNonMonotonic,
    resetDiscontinuity,
};

enum class SampleKind : std::uint32_t {
    applied,
    unchanged,
    clockUnavailable,
    generationMismatch,
    snapshotsMissing,
    invalid,
};

enum class DiscontinuityMode : std::uint32_t {
    reject,
    allowConsecutive,
};

struct SampleResult {
    SampleKind kind = SampleKind::clockUnavailable;
    Pose pose{};
};

class History final {
  public:
    static constexpr std::size_t kCapacity =
        clock::PresentationClock::kHistoryCapacity;

    void Reset() noexcept;
    [[nodiscard]] PublishKind Publish(std::uint64_t serial,
                                      std::uint64_t clockGeneration,
                                      const root::Matrix* matrices,
                                      std::size_t count,
                                      DiscontinuityMode discontinuityMode =
                                          DiscontinuityMode::reject) noexcept;
    [[nodiscard]] SampleResult Evaluate(
        const clock::Sample& clockSample) const noexcept;
    // The two stored poses a sample interpolates between, without copying.
    [[nodiscard]] bool Endpoints(const clock::Sample& clockSample,
                                 const Pose*& older,
                                 const Pose*& newer) const noexcept;

  private:
    struct Snapshot {
        std::uint64_t serial = 0;
        Pose pose{};
    };

    void Append(std::uint64_t serial, const root::Matrix* matrices,
                std::size_t count) noexcept;
    [[nodiscard]] const Snapshot& At(std::size_t index) const noexcept;

    std::array<Snapshot, kCapacity> snapshots_{};
    std::size_t first_ = 0;
    std::size_t count_ = 0;
    std::uint64_t generation_ = 0;
    bool hasGeneration_ = false;
};

static_assert(History::kCapacity == 2U,
              "Pose interpolation must retain only two ticks.");

} // namespace arb::pose
