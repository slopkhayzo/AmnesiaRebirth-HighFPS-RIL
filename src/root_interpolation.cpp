#include "root_interpolation.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace arb::root {
namespace {

struct Quaternion {
    float w = 1.0F;
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

float Dot(const Quaternion& left, const Quaternion& right) noexcept {
    return left.w * right.w + left.x * right.x + left.y * right.y +
           left.z * right.z;
}

Quaternion Normalize(const Quaternion& value) noexcept {
    const float lengthSquared = Dot(value, value);
    if (!(lengthSquared > 0.0F) || !std::isfinite(lengthSquared)) {
        return {};
    }
    const float inverseLength = 1.0F / std::sqrt(lengthSquared);
    return {value.w * inverseLength, value.x * inverseLength,
            value.y * inverseLength, value.z * inverseLength};
}

Quaternion FromNormalizedMatrix(const Matrix& matrix,
                                const float scale) noexcept {
    const auto element = [&matrix, scale](const std::size_t index) {
        return matrix.values[index] / scale;
    };
    const float m00 = element(0);
    const float m01 = element(1);
    const float m02 = element(2);
    const float m10 = element(4);
    const float m11 = element(5);
    const float m12 = element(6);
    const float m20 = element(8);
    const float m21 = element(9);
    const float m22 = element(10);
    Quaternion result{};
    const float trace = m00 + m11 + m22;
    if (trace > 0.0F) {
        const float factor = std::sqrt(trace + 1.0F) * 2.0F;
        result = {0.25F * factor, (m21 - m12) / factor,
                  (m02 - m20) / factor, (m10 - m01) / factor};
    } else if (m00 > m11 && m00 > m22) {
        const float factor = std::sqrt(1.0F + m00 - m11 - m22) * 2.0F;
        result = {(m21 - m12) / factor, 0.25F * factor,
                  (m01 + m10) / factor, (m02 + m20) / factor};
    } else if (m11 > m22) {
        const float factor = std::sqrt(1.0F + m11 - m00 - m22) * 2.0F;
        result = {(m02 - m20) / factor, (m01 + m10) / factor,
                  0.25F * factor, (m12 + m21) / factor};
    } else {
        const float factor = std::sqrt(1.0F + m22 - m00 - m11) * 2.0F;
        result = {(m10 - m01) / factor, (m02 + m20) / factor,
                  (m12 + m21) / factor, 0.25F * factor};
    }
    return Normalize(result);
}

Quaternion Slerp(const Quaternion& start, const Quaternion& finish,
                 const float alpha) noexcept {
    Quaternion end = finish;
    float cosine = Dot(start, end);
    if (cosine < 0.0F) {
        end = {-end.w, -end.x, -end.y, -end.z};
        cosine = -cosine;
    }
    cosine = std::clamp(cosine, -1.0F, 1.0F);
    if (cosine > 0.9995F) {
        return Normalize({start.w + alpha * (end.w - start.w),
                          start.x + alpha * (end.x - start.x),
                          start.y + alpha * (end.y - start.y),
                          start.z + alpha * (end.z - start.z)});
    }
    const float angle = std::acos(cosine);
    const float sine = std::sin(angle);
    const float startWeight = std::sin((1.0F - alpha) * angle) / sine;
    const float endWeight = std::sin(alpha * angle) / sine;
    return Normalize({start.w * startWeight + end.w * endWeight,
                      start.x * startWeight + end.x * endWeight,
                      start.y * startWeight + end.y * endWeight,
                      start.z * startWeight + end.z * endWeight});
}

void SetRotation(const Quaternion& input, const float scale,
                 Matrix& matrix) noexcept {
    const Quaternion q = Normalize(input);
    const float xx = q.x * q.x;
    const float yy = q.y * q.y;
    const float zz = q.z * q.z;
    const float xy = q.x * q.y;
    const float xz = q.x * q.z;
    const float yz = q.y * q.z;
    const float wx = q.w * q.x;
    const float wy = q.w * q.y;
    const float wz = q.w * q.z;
    matrix.values[0] = (1.0F - 2.0F * (yy + zz)) * scale;
    matrix.values[1] = (2.0F * (xy - wz)) * scale;
    matrix.values[2] = (2.0F * (xz + wy)) * scale;
    matrix.values[4] = (2.0F * (xy + wz)) * scale;
    matrix.values[5] = (1.0F - 2.0F * (xx + zz)) * scale;
    matrix.values[6] = (2.0F * (yz - wx)) * scale;
    matrix.values[8] = (2.0F * (xz - wy)) * scale;
    matrix.values[9] = (2.0F * (yz + wx)) * scale;
    matrix.values[10] = (1.0F - 2.0F * (xx + yy)) * scale;
}

bool IsDiscontinuous(const Matrix& older, const Matrix& newer) noexcept {
    const float dx = newer.values[3] - older.values[3];
    const float dy = newer.values[7] - older.values[7];
    const float dz = newer.values[11] - older.values[11];
    const float limit = History::kMaximumPositionDeltaPerTick;
    return dx * dx + dy * dy + dz * dz > limit * limit ||
           OrientationDistance(older, newer) >
               History::kMaximumOrientationDeltaPerTick;
}

} // namespace

Matrix Identity(const float scale) noexcept {
    Matrix result{};
    result.values[0] = scale;
    result.values[5] = scale;
    result.values[10] = scale;
    result.values[15] = 1.0F;
    return result;
}

bool Analyze(const Matrix& matrix, TransformInfo& info) noexcept {
    for (const float value : matrix.values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    constexpr float affineEpsilon = 2.0e-3F;
    if (std::abs(matrix.values[12]) > affineEpsilon ||
        std::abs(matrix.values[13]) > affineEpsilon ||
        std::abs(matrix.values[14]) > affineEpsilon ||
        std::abs(matrix.values[15] - 1.0F) > affineEpsilon) {
        return false;
    }
    std::array<float, 3> lengths{};
    for (std::size_t row = 0; row < 3; ++row) {
        float squared = 0.0F;
        for (std::size_t column = 0; column < 3; ++column) {
            const float value = matrix.values[row * 4U + column];
            squared += value * value;
        }
        lengths[row] = std::sqrt(squared);
    }
    const float scale = (lengths[0] + lengths[1] + lengths[2]) / 3.0F;
    if (!std::isfinite(scale) || scale < 1.0e-4F || scale > 16.0F) {
        return false;
    }
    constexpr float basisEpsilon = 3.0e-3F;
    for (const float length : lengths) {
        if (std::abs(length - scale) > basisEpsilon * std::max(1.0F, scale)) {
            return false;
        }
    }
    for (std::size_t first = 0; first < 3; ++first) {
        for (std::size_t second = first + 1U; second < 3; ++second) {
            float dot = 0.0F;
            for (std::size_t column = 0; column < 3; ++column) {
                dot += matrix.values[first * 4U + column] *
                       matrix.values[second * 4U + column];
            }
            if (std::abs(dot / (scale * scale)) > basisEpsilon) {
                return false;
            }
        }
    }
    const float determinant =
        matrix.values[0] * (matrix.values[5] * matrix.values[10] -
                            matrix.values[6] * matrix.values[9]) -
        matrix.values[1] * (matrix.values[4] * matrix.values[10] -
                            matrix.values[6] * matrix.values[8]) +
        matrix.values[2] * (matrix.values[4] * matrix.values[9] -
                            matrix.values[5] * matrix.values[8]);
    if (std::abs(determinant / (scale * scale * scale) - 1.0F) > 5.0e-3F) {
        return false;
    }
    info.uniformScale = scale;
    return true;
}

bool NearlyEqual(const Matrix& left, const Matrix& right,
                 const float epsilon) noexcept {
    for (std::size_t index = 0; index < left.values.size(); ++index) {
        if (std::abs(left.values[index] - right.values[index]) > epsilon) {
            return false;
        }
    }
    return true;
}

float OrientationDistance(const Matrix& left, const Matrix& right) noexcept {
    TransformInfo leftInfo{};
    TransformInfo rightInfo{};
    if (!Analyze(left, leftInfo) || !Analyze(right, rightInfo)) {
        return 3.14159265358979323846F;
    }
    const Quaternion first = FromNormalizedMatrix(left, leftInfo.uniformScale);
    Quaternion second = FromNormalizedMatrix(right, rightInfo.uniformScale);
    if (Dot(first, second) < 0.0F) {
        second = {-second.w, -second.x, -second.y, -second.z};
    }
    // The chord between the unit quaternions is exact for identical inputs;
    // acos(dot) is not, because it loses all precision near 1.
    const Quaternion difference{first.w - second.w, first.x - second.x,
                                first.y - second.y, first.z - second.z};
    const float halfChord =
        std::min(0.5F * std::sqrt(Dot(difference, difference)), 1.0F);
    return 4.0F * std::asin(halfChord);
}

bool Interpolate(const Matrix& older, const Matrix& newer, const double alpha,
                 Matrix& result) noexcept {
    TransformInfo olderInfo{};
    TransformInfo newerInfo{};
    if (!Analyze(older, olderInfo) || !Analyze(newer, newerInfo) ||
        !std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0) {
        return false;
    }
    const float t = static_cast<float>(alpha);
    const float scale = olderInfo.uniformScale +
        (newerInfo.uniformScale - olderInfo.uniformScale) * t;
    result = Identity(scale);
    SetRotation(Slerp(FromNormalizedMatrix(older, olderInfo.uniformScale),
                      FromNormalizedMatrix(newer, newerInfo.uniformScale), t),
                scale, result);
    result.values[3] = older.values[3] + (newer.values[3] - older.values[3]) * t;
    result.values[7] = older.values[7] + (newer.values[7] - older.values[7]) * t;
    result.values[11] = older.values[11] + (newer.values[11] - older.values[11]) * t;
    TransformInfo resultInfo{};
    return Analyze(result, resultInfo);
}

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
                     const Matrix& matrix) noexcept {
    if (count_ < kCapacity) {
        snapshots_[(first_ + count_) % kCapacity] = {serial, matrix};
        ++count_;
    } else {
        snapshots_[first_] = {serial, matrix};
        first_ = (first_ + 1U) % kCapacity;
    }
}

PublishKind History::Publish(const std::uint64_t serial,
                             const std::uint64_t clockGeneration,
                             const Matrix& matrix) noexcept {
    TransformInfo info{};
    if (!Analyze(matrix, info)) {
        Reset();
        return PublishKind::invalid;
    }
    if (!hasGeneration_ || generation_ != clockGeneration) {
        Reset();
        generation_ = clockGeneration;
        hasGeneration_ = true;
        Append(serial, matrix);
        return PublishKind::resetGeneration;
    }
    if (count_ != 0U) {
        const Snapshot& latest = At(count_ - 1U);
        if (serial < latest.serial) {
            Reset();
            generation_ = clockGeneration;
            hasGeneration_ = true;
            Append(serial, matrix);
            return PublishKind::resetNonMonotonic;
        }
        if (serial == latest.serial) {
            if (NearlyEqual(latest.matrix, matrix)) {
                return PublishKind::unchanged;
            }
            Reset();
            generation_ = clockGeneration;
            hasGeneration_ = true;
            Append(serial, matrix);
            return PublishKind::resetSameTickMutation;
        }
        if (serial != latest.serial + 1U ||
            IsDiscontinuous(latest.matrix, matrix)) {
            Reset();
            generation_ = clockGeneration;
            hasGeneration_ = true;
            Append(serial, matrix);
            return PublishKind::resetDiscontinuity;
        }
    }
    Append(serial, matrix);
    return PublishKind::appended;
}

bool History::Endpoints(const clock::Sample& sample, Matrix& older,
                        Matrix& newer) const noexcept {
    if (sample.kind != clock::SampleKind::interpolated ||
        sample.olderSerial == sample.newerSerial || !hasGeneration_ ||
        generation_ != sample.generation) {
        return false;
    }
    bool hasOlder = false;
    bool hasNewer = false;
    for (std::size_t index = 0; index < count_; ++index) {
        const Snapshot& candidate = At(index);
        if (candidate.serial == sample.olderSerial) {
            older = candidate.matrix;
            hasOlder = true;
        }
        if (candidate.serial == sample.newerSerial) {
            newer = candidate.matrix;
            hasNewer = true;
        }
    }
    return hasOlder && hasNewer;
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
        if (candidate.serial == sample.olderSerial) older = &candidate;
        if (candidate.serial == sample.newerSerial) newer = &candidate;
    }
    if (older == nullptr || newer == nullptr) {
        return {SampleKind::snapshotsMissing, {}};
    }
    // Identical endpoints already equal the authoritative transform, so the
    // caller normally has nothing to substitute or restore. The resting
    // matrix is still returned for callers that add a view-space offset.
    if (std::memcmp(older->matrix.values.data(), newer->matrix.values.data(),
                    sizeof(older->matrix.values)) == 0) {
        return {SampleKind::unchanged, newer->matrix};
    }
    Matrix interpolated{};
    if (!Interpolate(older->matrix, newer->matrix, sample.alpha,
                     interpolated)) {
        return {SampleKind::invalid, {}};
    }
    return {SampleKind::applied, interpolated};
}

} // namespace arb::root
