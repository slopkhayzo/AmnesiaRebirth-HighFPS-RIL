#include "camera_attachment.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

using arb::root::Matrix;

void Require(const bool condition, const char* const message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

Matrix RotateY(const float angle) {
    Matrix matrix = arb::root::Identity();
    matrix.values[0] = std::cos(angle);
    matrix.values[2] = std::sin(angle);
    matrix.values[8] = -std::sin(angle);
    matrix.values[10] = std::cos(angle);
    return matrix;
}

Matrix RotateX(const float angle) {
    Matrix matrix = arb::root::Identity();
    matrix.values[5] = std::cos(angle);
    matrix.values[6] = -std::sin(angle);
    matrix.values[9] = std::sin(angle);
    matrix.values[10] = std::cos(angle);
    return matrix;
}

Matrix Translate(const float x, const float y, const float z) {
    Matrix matrix = arb::root::Identity();
    matrix.values[3] = x;
    matrix.values[7] = y;
    matrix.values[11] = z;
    return matrix;
}

Matrix Transposed(const Matrix& matrix) {
    Matrix result = matrix;
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            result.values[row * 4U + column] =
                matrix.values[column * 4U + row];
        }
    }
    return result;
}

float MaxDifference(const Matrix& left, const Matrix& right) {
    float difference = 0.0F;
    for (std::size_t index = 0; index < 12; ++index) {
        difference = std::max(
            difference, std::abs(left.values[index] - right.values[index]));
    }
    return difference;
}

// A three-joint chain: hips -> spine -> head, each offset from its parent.
struct Chain {
    std::array<Matrix, 3> locals;
    std::array<std::int16_t, 3> parents{-1, 0, 1};
};

Chain MakeChain(const float hipsYaw, const float spinePitch,
                const float headYaw) {
    Chain chain;
    chain.locals[0] = arb::attach::Multiply(Translate(0.0F, 1.0F, 0.0F),
                                            RotateY(hipsYaw));
    chain.locals[1] = arb::attach::Multiply(Translate(0.0F, 0.4F, 0.1F),
                                            RotateX(spinePitch));
    chain.locals[2] = arb::attach::Multiply(Translate(0.0F, 0.3F, 0.0F),
                                            RotateY(headYaw));
    return chain;
}

arb::attach::CameraEndpoint CameraOnHead(const Chain& chain,
                                         const Matrix& root,
                                         const Matrix& offset,
                                         const bool storeTransposed) {
    Matrix head{};
    Require(arb::attach::ComposeBone(chain.locals.data(),
                                     chain.parents.data(), 3, 2, head),
            "chain composition failed");
    const Matrix world = arb::attach::Multiply(
        arb::attach::Multiply(root, head), offset);
    arb::attach::CameraEndpoint camera{};
    camera.rotation = storeTransposed ? Transposed(world) : world;
    camera.rotation.values[3] = 0.0F;
    camera.rotation.values[7] = 0.0F;
    camera.rotation.values[11] = 0.0F;
    camera.x = world.values[3];
    camera.y = world.values[7];
    camera.z = world.values[11];
    return camera;
}

void TestInverseAndCompose() {
    const Matrix matrix = arb::attach::Multiply(Translate(3.0F, -2.0F, 5.0F),
                                                RotateY(0.7F));
    Matrix inverse{};
    Require(arb::attach::InvertUniformRigid(matrix, inverse),
            "rigid inverse was rejected");
    Require(MaxDifference(arb::attach::Multiply(matrix, inverse),
                          arb::root::Identity()) < 1.0e-5F,
            "inverse did not undo the transform");

    const Chain chain = MakeChain(0.3F, -0.2F, 0.5F);
    Matrix head{};
    Require(arb::attach::ComposeBone(chain.locals.data(),
                                     chain.parents.data(), 3, 2, head),
            "bone composition failed");
    const Matrix expected = arb::attach::Multiply(
        arb::attach::Multiply(chain.locals[0], chain.locals[1]),
        chain.locals[2]);
    Require(MaxDifference(head, expected) < 1.0e-6F,
            "composed bone differs from the explicit chain product");
}

void RunAttachmentCase(const bool storeTransposed) {
    const Chain older = MakeChain(0.10F, -0.30F, 0.20F);
    const Chain newer = MakeChain(0.45F, 0.15F, -0.25F);
    const Matrix olderRoot = Translate(10.0F, 0.0F, -4.0F);
    const Matrix newerRoot = Translate(10.2F, 0.0F, -4.1F);
    const Matrix offset = arb::attach::Multiply(Translate(0.0F, 0.1F, 0.12F),
                                                RotateX(0.05F));
    const auto olderCamera =
        CameraOnHead(older, olderRoot, offset, storeTransposed);
    const auto newerCamera =
        CameraOnHead(newer, newerRoot, offset, storeTransposed);

    const arb::attach::Attachment attachment = arb::attach::FindAttachment(
        older.locals.data(), newer.locals.data(), older.parents.data(), 3,
        olderRoot, newerRoot, olderCamera, newerCamera, 2.0e-3F);
    Require(attachment.valid && attachment.bone == 2U,
            "the head attachment was not found");
    Require(attachment.transposed == storeTransposed,
            "the camera storage convention was misidentified");

    // Present on a mid-tick skeleton and compare with the camera rebuilt
    // from that same skeleton.
    const Chain middle = MakeChain(0.27F, -0.08F, -0.02F);
    const Matrix middleRoot = Translate(10.1F, 0.0F, -4.05F);
    Matrix head{};
    Require(arb::attach::ComposeBone(middle.locals.data(),
                                     middle.parents.data(), 3, 2, head),
            "mid-tick composition failed");
    arb::attach::CameraEndpoint presented = olderCamera;
    Require(arb::attach::PresentCamera(
                attachment, arb::attach::Multiply(middleRoot, head), 0.5,
                presented),
            "presenting on the skeleton failed");
    const auto expected =
        CameraOnHead(middle, middleRoot, offset, storeTransposed);
    Require(MaxDifference(presented.rotation, expected.rotation) < 2.0e-4F &&
                std::abs(presented.x - expected.x) < 2.0e-4F &&
                std::abs(presented.y - expected.y) < 2.0e-4F &&
                std::abs(presented.z - expected.z) < 2.0e-4F,
            "presented camera left the bone it is attached to");
}

void TestFreeCameraIsNotAttached() {
    const Chain older = MakeChain(0.10F, -0.30F, 0.20F);
    const Chain newer = MakeChain(0.45F, 0.15F, -0.25F);
    const Matrix root = arb::root::Identity();
    // The camera stays put while every bone moves.
    arb::attach::CameraEndpoint camera{};
    camera.rotation = RotateY(0.3F);
    camera.x = 0.5F;
    camera.y = 1.6F;
    camera.z = 0.2F;
    const arb::attach::Attachment attachment = arb::attach::FindAttachment(
        older.locals.data(), newer.locals.data(), older.parents.data(), 3,
        root, root, camera, camera, 2.0e-3F);
    Require(!attachment.valid,
            "a camera unrelated to the skeleton was attached to it");

    arb::attach::CameraEndpoint untouched = camera;
    Require(!arb::attach::PresentCamera(attachment, root, 0.5, untouched),
            "an invalid attachment still produced a camera");
}

void TestInvalidInputFailsClosed() {
    Matrix sheared = arb::root::Identity();
    sheared.values[1] = 0.4F;
    Matrix inverse{};
    Require(!arb::attach::InvertUniformRigid(sheared, inverse),
            "a sheared matrix was inverted as rigid");
    const std::array<Matrix, 2> locals{arb::root::Identity(),
                                       arb::root::Identity()};
    const std::array<std::int16_t, 2> cyclic{1, 0};
    Matrix model{};
    Require(!arb::attach::ComposeBone(locals.data(), cyclic.data(), 2, 0,
                                      model),
            "a parent that is not earlier in the list was followed");
}

void TestPreviousBoneIsKept() {
    // The head does not move relative to the spine, so both qualify.
    const Chain older = MakeChain(0.10F, -0.30F, 0.20F);
    const Chain newer = MakeChain(0.45F, 0.15F, 0.20F);
    const Matrix root = Translate(1.0F, 0.0F, 2.0F);
    const Matrix offset = Translate(0.0F, 0.1F, 0.12F);
    const auto olderCamera = CameraOnHead(older, root, offset, false);
    const auto newerCamera = CameraOnHead(newer, root, offset, false);
    const arb::attach::Attachment fresh = arb::attach::FindAttachment(
        older.locals.data(), newer.locals.data(), older.parents.data(), 3,
        root, root, olderCamera, newerCamera, 2.0e-3F);
    Require(fresh.valid && fresh.bone == 1U,
            "without history the shallowest qualifying bone should win");
    arb::attach::Attachment previous{};
    previous.valid = true;
    previous.bone = 2U;
    const arb::attach::Attachment kept = arb::attach::FindAttachment(
        older.locals.data(), newer.locals.data(), older.parents.data(), 3,
        root, root, olderCamera, newerCamera, 2.0e-3F, previous);
    Require(kept.valid && kept.bone == 2U,
            "a still-qualifying previous bone was abandoned");
    previous.bone = 0U;
    const arb::attach::Attachment moved = arb::attach::FindAttachment(
        older.locals.data(), newer.locals.data(), older.parents.data(), 3,
        root, root, olderCamera, newerCamera, 2.0e-3F, previous);
    Require(moved.valid && moved.bone == 1U,
            "a previous bone that no longer qualifies was kept");
}

} // namespace

int main() {
    TestInverseAndCompose();
    RunAttachmentCase(false);
    RunAttachmentCase(true);
    TestFreeCameraIsNotAttached();
    TestPreviousBoneIsKept();
    TestInvalidInputFailsClosed();
    std::cout << "camera attachment tests passed\n";
    return EXIT_SUCCESS;
}
