#pragma once

#include "presentation_clock.h"
#include "root_interpolation.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace arb::camera {

struct Vec3 {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

enum class RotateMode : std::uint32_t {
    eulerAngles = 0,
    matrix = 1,
};

struct Pose {
    Vec3 position{};
    float pitch = 0.0F;
    float yaw = 0.0F;
    float roll = 0.0F;
    RotateMode rotateMode = RotateMode::eulerAngles;
    root::Matrix rotationMatrix{};
};

enum class PublishKind : std::uint32_t {
    appended,
    unchanged,
    invalid,
    resetGeneration,
    resetNonMonotonic,
    updatedSameTick,
    resetDiscontinuity,
};

enum class SampleKind : std::uint32_t {
    applied,
    clockUnavailable,
    generationMismatch,
    snapshotsMissing,
    invalid,
};

struct PublishResult {
    PublishKind kind = PublishKind::invalid;
};

struct SampleResult {
    SampleKind kind = SampleKind::clockUnavailable;
    Pose pose{};
};

class History final {
  public:
    static constexpr std::size_t kCapacity =
        clock::PresentationClock::kHistoryCapacity;
    static constexpr float kMaximumPositionDeltaPerTick = 5.0F;
    static constexpr float kMaximumOrientationDeltaPerTick = 2.09439510239F;

    void Reset() noexcept;
    [[nodiscard]] PublishResult Publish(std::uint64_t serial,
                                        std::uint64_t clockGeneration,
                                        const Pose& pose) noexcept;
    [[nodiscard]] SampleResult Evaluate(
        const clock::Sample& clockSample) const noexcept;
    // The two authoritative poses a sample interpolates between.
    [[nodiscard]] bool Endpoints(const clock::Sample& clockSample,
                                 Pose& older, Pose& newer) const noexcept;
    [[nodiscard]] std::size_t SnapshotCount() const noexcept { return count_; }

  private:
    struct Snapshot {
        std::uint64_t serial = 0;
        Pose pose{};
    };

    void Append(std::uint64_t serial, const Pose& pose) noexcept;
    void ReplaceLatest(const Pose& pose) noexcept;
    [[nodiscard]] const Snapshot& At(std::size_t logicalIndex) const noexcept;

    std::array<Snapshot, kCapacity> snapshots_{};
    std::size_t first_ = 0;
    std::size_t count_ = 0;
    std::uint64_t generation_ = 0;
    bool hasGeneration_ = false;
};

static_assert(History::kCapacity == 2U,
              "Camera interpolation must retain only two ticks.");

[[nodiscard]] bool IsFinite(const Pose& pose) noexcept;
[[nodiscard]] bool NearlyEqual(const Pose& left, const Pose& right,
                               float epsilon = 1.0e-5F) noexcept;
[[nodiscard]] bool Interpolate(const Pose& older, const Pose& newer,
                               double alpha, Pose& result) noexcept;
[[nodiscard]] float OrientationDistance(const Pose& left,
                                        const Pose& right) noexcept;
[[nodiscard]] const wchar_t* PublishKindName(PublishKind kind) noexcept;
[[nodiscard]] const wchar_t* SampleKindName(SampleKind kind) noexcept;

} // namespace arb::camera
