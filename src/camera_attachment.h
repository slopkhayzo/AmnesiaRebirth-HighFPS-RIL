#pragma once

#include "root_interpolation.h"

#include <cstddef>
#include <cstdint>

// During scripted player animations the game drives the camera from a bone
// of the player's skeleton. Interpolating the camera's world pose and the
// skeleton's bone-local poses independently makes them disagree between
// ticks (a chain of interpolated joints does not follow the straight line
// between its end poses), so geometry near the camera appears to shake at
// the tick rate. These helpers detect a bone the camera is rigidly attached
// to across a tick pair and place the presented camera on the interpolated
// skeleton instead.
namespace arb::attach {

struct CameraEndpoint {
    // The camera's rotation matrix exactly as the engine stores it.
    root::Matrix rotation{};
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

struct Attachment {
    bool valid = false;
    std::size_t bone = 0;
    // Whether the stored camera rotation is the transpose of its world
    // orientation (a view rotation) rather than the orientation itself.
    bool transposed = false;
    root::Matrix olderOffset{};
    root::Matrix newerOffset{};
    float error = 0.0F;
};

[[nodiscard]] root::Matrix Multiply(const root::Matrix& left,
                                    const root::Matrix& right) noexcept;
[[nodiscard]] bool InvertUniformRigid(const root::Matrix& matrix,
                                      root::Matrix& inverse) noexcept;

// Model-space matrix of one bone from parent-first local matrices.
// `parents[i]` is the index of bone i's parent, or negative for a root.
[[nodiscard]] bool ComposeBone(const root::Matrix* locals,
                               const std::int16_t* parents, std::size_t count,
                               std::size_t bone, root::Matrix& model) noexcept;

// Searches for the bone whose camera-relative offset is the same at both
// ticks to within `tolerance`. When `previous` is valid and its bone still
// qualifies, that bone is kept: several nearly rigid bones (head, jaw,
// eyes) can qualify, and switching between them from tick to tick moves
// the presented camera onto a slightly different path at each switch.
[[nodiscard]] Attachment FindAttachment(
    const root::Matrix* olderLocals, const root::Matrix* newerLocals,
    const std::int16_t* parents, std::size_t count,
    const root::Matrix& olderRoot, const root::Matrix& newerRoot,
    const CameraEndpoint& olderCamera, const CameraEndpoint& newerCamera,
    float tolerance, const Attachment& previous = {}) noexcept;

// Camera pose for the attached bone's interpolated world matrix. On entry
// `camera.rotation` is a template in the engine's storage form; only its
// 3x3 rotation and the position are replaced.
[[nodiscard]] bool PresentCamera(const Attachment& attachment,
                                 const root::Matrix& boneWorld, double alpha,
                                 CameraEndpoint& camera) noexcept;

} // namespace arb::attach
