#include "pose_interpolation.h"

#include <cmath>
#include <cstring>

namespace arb::pose {
namespace {

bool IsDiscontinuous(const root::Matrix& older,
                     const root::Matrix& newer) noexcept {
    const float dx = newer.values[3] - older.values[3];
    const float dy = newer.values[7] - older.values[7];
    const float dz = newer.values[11] - older.values[11];
    const float positionLimit = root::History::kMaximumPositionDeltaPerTick;
    return dx * dx + dy * dy + dz * dz > positionLimit * positionLimit ||
           root::OrientationDistance(older, newer) >
               root::History::kMaximumOrientationDeltaPerTick;
}

bool ValidPose(const root::Matrix* const matrices,
               const std::size_t count) noexcept {
    if (matrices == nullptr || count == 0U || count > kMaximumBones) {
        return false;
    }
    for (std::size_t index = 0; index < count; ++index) {
        root::TransformInfo info{};
        if (!root::Analyze(matrices[index], info)) {
            return false;
        }
    }
    return true;
}

} // namespace

void History::Reset() noexcept {
    first_ = 0;
    count_ = 0;
    generation_ = 0;
    hasGeneration_ = false;
}

const History::Snapshot& History::At(const std::size_t index) const noexcept {
    return snapshots_[(first_ + index) % kCapacity];
}

void History::Append(const std::uint64_t serial,
                     const root::Matrix* const matrices,
                     const std::size_t count) noexcept {
    Snapshot* destination = nullptr;
    if (count_ < kCapacity) {
        destination = &snapshots_[(first_ + count_) % kCapacity];
        ++count_;
    } else {
        destination = &snapshots_[first_];
        first_ = (first_ + 1U) % kCapacity;
    }
    destination->serial = serial;
    destination->pose.count = count;
    for (std::size_t index = 0; index < count; ++index) {
        destination->pose.matrices[index] = matrices[index];
    }
}

PublishKind History::Publish(const std::uint64_t serial,
                             const std::uint64_t clockGeneration,
                             const root::Matrix* const matrices,
                             const std::size_t count,
                             const DiscontinuityMode discontinuityMode) noexcept {
    if (!ValidPose(matrices, count)) {
        Reset();
        return PublishKind::invalid;
    }
    if (!hasGeneration_ || generation_ != clockGeneration) {
        Reset();
        generation_ = clockGeneration;
        hasGeneration_ = true;
        Append(serial, matrices, count);
        return PublishKind::resetGeneration;
    }
    if (count_ != 0U) {
        const Snapshot& latest = At(count_ - 1U);
        if (serial <= latest.serial) {
            Reset();
            generation_ = clockGeneration;
            hasGeneration_ = true;
            Append(serial, matrices, count);
            return PublishKind::resetNonMonotonic;
        }
        bool discontinuity = serial != latest.serial + 1U ||
                             latest.pose.count != count;
        if (discontinuityMode == DiscontinuityMode::reject) {
            for (std::size_t index = 0;
                 index < count && !discontinuity; ++index) {
                discontinuity = IsDiscontinuous(
                    latest.pose.matrices[index], matrices[index]);
            }
        }
        if (discontinuity) {
            Reset();
            generation_ = clockGeneration;
            hasGeneration_ = true;
            Append(serial, matrices, count);
            return PublishKind::resetDiscontinuity;
        }
    }
    Append(serial, matrices, count);
    return PublishKind::appended;
}

bool History::Endpoints(const clock::Sample& sample, const Pose*& older,
                        const Pose*& newer) const noexcept {
    older = nullptr;
    newer = nullptr;
    if (sample.kind != clock::SampleKind::interpolated ||
        sample.olderSerial == sample.newerSerial || !hasGeneration_ ||
        generation_ != sample.generation) {
        return false;
    }
    for (std::size_t index = 0; index < count_; ++index) {
        const Snapshot& candidate = At(index);
        if (candidate.serial == sample.olderSerial) {
            older = &candidate.pose;
        }
        if (candidate.serial == sample.newerSerial) {
            newer = &candidate.pose;
        }
    }
    return older != nullptr && newer != nullptr &&
           older->count == newer->count;
}

SampleResult History::Evaluate(const clock::Sample& sample) const noexcept {
    if (sample.kind != clock::SampleKind::interpolated ||
        sample.olderSerial == sample.newerSerial) {
        return {SampleKind::clockUnavailable, {}};
    }
    if (!hasGeneration_ || generation_ != sample.generation) {
        return {SampleKind::generationMismatch, {}};
    }
    const Snapshot* older = nullptr;
    const Snapshot* newer = nullptr;
    for (std::size_t index = 0; index < count_; ++index) {
        const Snapshot& candidate = At(index);
        if (candidate.serial == sample.olderSerial) {
            older = &candidate;
        }
        if (candidate.serial == sample.newerSerial) {
            newer = &candidate;
        }
    }
    if (older == nullptr || newer == nullptr ||
        older->pose.count != newer->pose.count) {
        return {SampleKind::snapshotsMissing, {}};
    }
    const std::size_t count = older->pose.count;
    // An identical pose pair already equals the authoritative skeleton.
    if (std::memcmp(older->pose.matrices.data(), newer->pose.matrices.data(),
                    count * sizeof(root::Matrix)) == 0) {
        return {SampleKind::unchanged, {}};
    }
    SampleResult result{};
    result.kind = SampleKind::applied;
    result.pose.count = count;
    for (std::size_t index = 0; index < count; ++index) {
        const root::Matrix& from = older->pose.matrices[index];
        const root::Matrix& to = newer->pose.matrices[index];
        // Bones that did not move this tick keep their exact stored matrix
        // and skip the quaternion round trip.
        if (std::memcmp(from.values.data(), to.values.data(),
                        sizeof(from.values)) == 0) {
            result.pose.matrices[index] = to;
        } else if (!root::Interpolate(from, to, sample.alpha,
                                      result.pose.matrices[index])) {
            return {SampleKind::invalid, {}};
        }
    }
    return result;
}

} // namespace arb::pose
