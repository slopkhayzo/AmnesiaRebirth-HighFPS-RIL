#include "camera_attachment.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace arb::attach {
namespace {

constexpr std::size_t kMaximumBones = 256U;

root::Matrix Transposed3x3(const root::Matrix& matrix) noexcept {
    root::Matrix result = matrix;
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            result.values[row * 4U + column] =
                matrix.values[column * 4U + row];
        }
    }
    return result;
}

root::Matrix CameraWorld(const CameraEndpoint& camera,
                         const bool transposed) noexcept {
    root::Matrix world =
        transposed ? Transposed3x3(camera.rotation) : camera.rotation;
    world.values[3] = camera.x;
    world.values[7] = camera.y;
    world.values[11] = camera.z;
    world.values[12] = 0.0F;
    world.values[13] = 0.0F;
    world.values[14] = 0.0F;
    world.values[15] = 1.0F;
    return world;
}

// Largest difference between two offsets; translation is weighted down so
// one tolerance serves both rotation terms and world-unit distances.
float OffsetDifference(const root::Matrix& left,
                       const root::Matrix& right) noexcept {
    float difference = 0.0F;
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            const std::size_t index = row * 4U + column;
            difference = std::max(
                difference, std::abs(left.values[index] - right.values[index]));
        }
        const std::size_t translation = row * 4U + 3U;
        difference = std::max(
            difference, 0.5F * std::abs(left.values[translation] -
                                        right.values[translation]));
    }
    return difference;
}

bool ComposeAll(const root::Matrix* const locals,
                const std::int16_t* const parents, const std::size_t count,
                std::array<root::Matrix, kMaximumBones>& models) noexcept {
    for (std::size_t index = 0; index < count; ++index) {
        const std::int16_t parent = parents[index];
        if (parent < 0) {
            models[index] = locals[index];
        } else if (static_cast<std::size_t>(parent) < index) {
            models[index] = Multiply(models[static_cast<std::size_t>(parent)],
                                     locals[index]);
        } else {
            return false;
        }
    }
    return true;
}

} // namespace

root::Matrix Multiply(const root::Matrix& left,
                      const root::Matrix& right) noexcept {
    root::Matrix result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            float sum = column == 3U ? left.values[row * 4U + 3U] : 0.0F;
            for (std::size_t inner = 0; inner < 3; ++inner) {
                sum += left.values[row * 4U + inner] *
                       right.values[inner * 4U + column];
            }
            result.values[row * 4U + column] = sum;
        }
    }
    result.values[15] = 1.0F;
    return result;
}

bool InvertUniformRigid(const root::Matrix& matrix,
                        root::Matrix& inverse) noexcept {
    root::TransformInfo info{};
    if (!root::Analyze(matrix, info)) {
        return false;
    }
    const float inverseSquare = 1.0F / (info.uniformScale * info.uniformScale);
    inverse = {};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            inverse.values[row * 4U + column] =
                matrix.values[column * 4U + row] * inverseSquare;
        }
    }
    for (std::size_t row = 0; row < 3; ++row) {
        inverse.values[row * 4U + 3U] =
            -(inverse.values[row * 4U] * matrix.values[3] +
              inverse.values[row * 4U + 1U] * matrix.values[7] +
              inverse.values[row * 4U + 2U] * matrix.values[11]);
    }
    inverse.values[15] = 1.0F;
    return true;
}

bool ComposeBone(const root::Matrix* const locals,
                 const std::int16_t* const parents, const std::size_t count,
                 const std::size_t bone, root::Matrix& model) noexcept {
    if (locals == nullptr || parents == nullptr || bone >= count ||
        count > kMaximumBones) {
        return false;
    }
    model = locals[bone];
    std::size_t current = bone;
    for (std::size_t depth = 0; depth < count; ++depth) {
        const std::int16_t parent = parents[current];
        if (parent < 0) {
            return true;
        }
        if (static_cast<std::size_t>(parent) >= current) {
            return false;
        }
        current = static_cast<std::size_t>(parent);
        model = Multiply(locals[current], model);
    }
    return false;
}

Attachment FindAttachment(const root::Matrix* const olderLocals,
                          const root::Matrix* const newerLocals,
                          const std::int16_t* const parents,
                          const std::size_t count,
                          const root::Matrix& olderRoot,
                          const root::Matrix& newerRoot,
                          const CameraEndpoint& olderCamera,
                          const CameraEndpoint& newerCamera,
                          const float tolerance,
                          const Attachment& previous) noexcept {
    Attachment best{};
    Attachment kept{};
    if (olderLocals == nullptr || newerLocals == nullptr ||
        parents == nullptr || count == 0U || count > kMaximumBones) {
        return best;
    }
    std::array<root::Matrix, kMaximumBones> olderModels;
    std::array<root::Matrix, kMaximumBones> newerModels;
    if (!ComposeAll(olderLocals, parents, count, olderModels) ||
        !ComposeAll(newerLocals, parents, count, newerModels)) {
        return best;
    }
    for (const bool transposed : {false, true}) {
        const root::Matrix olderWorld = CameraWorld(olderCamera, transposed);
        const root::Matrix newerWorld = CameraWorld(newerCamera, transposed);
        for (std::size_t bone = 0; bone < count; ++bone) {
            root::Matrix olderInverse{};
            root::Matrix newerInverse{};
            if (!InvertUniformRigid(Multiply(olderRoot, olderModels[bone]),
                                    olderInverse) ||
                !InvertUniformRigid(Multiply(newerRoot, newerModels[bone]),
                                    newerInverse)) {
                continue;
            }
            const root::Matrix olderOffset =
                Multiply(olderInverse, olderWorld);
            const root::Matrix newerOffset =
                Multiply(newerInverse, newerWorld);
            const float error = OffsetDifference(olderOffset, newerOffset);
            if (previous.valid && bone == previous.bone &&
                transposed == previous.transposed && std::isfinite(error) &&
                error < tolerance) {
                kept.valid = true;
                kept.bone = bone;
                kept.transposed = transposed;
                kept.olderOffset = olderOffset;
                kept.newerOffset = newerOffset;
                kept.error = error;
            }
            // Prefer the first (shallowest) bone unless a later one is
            // clearly tighter, so a still frame does not flip candidates.
            if (std::isfinite(error) && error < tolerance &&
                (!best.valid || error < best.error * 0.5F)) {
                best.valid = true;
                best.bone = bone;
                best.transposed = transposed;
                best.olderOffset = olderOffset;
                best.newerOffset = newerOffset;
                best.error = error;
            }
        }
    }
    return kept.valid ? kept : best;
}

bool PresentCamera(const Attachment& attachment,
                   const root::Matrix& boneWorld, const double alpha,
                   CameraEndpoint& camera) noexcept {
    if (!attachment.valid) {
        return false;
    }
    root::Matrix offset{};
    if (!root::Interpolate(attachment.olderOffset, attachment.newerOffset,
                           alpha, offset)) {
        offset = attachment.newerOffset;
    }
    const root::Matrix world = Multiply(boneWorld, offset);
    root::TransformInfo info{};
    if (!root::Analyze(world, info)) {
        return false;
    }
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            camera.rotation.values[row * 4U + column] =
                attachment.transposed ? world.values[column * 4U + row]
                                      : world.values[row * 4U + column];
        }
    }
    camera.x = world.values[3];
    camera.y = world.values[7];
    camera.z = world.values[11];
    return true;
}

} // namespace arb::attach
