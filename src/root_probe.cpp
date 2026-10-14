#include "root_probe.h"

#include "logger.h"
#include "pose_interpolation.h"
#include "root_interpolation.h"

#include <MinHook.h>
#include <intrin.h>

#include <array>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>

namespace arb::root_probe {
namespace {

constexpr std::uintptr_t kEntitySetMatrixRva = 0x00155460U;
constexpr std::uintptr_t kEntitySetVarBoolRva = 0x00155CC0U;
constexpr std::uintptr_t kEntityGetMatrixRva = 0x002215E0U;
constexpr std::uintptr_t kEntityVirtualSetMatrixRva = 0x0022ADE0U;
constexpr std::uintptr_t kMatrixInverseRva = 0x002BC590U;
constexpr std::uintptr_t kMatrixMultiplyRva = 0x002BCA60U;
constexpr std::uintptr_t kNodeMatrixChangedRva = 0x00309C90U;
constexpr std::uintptr_t kBoneSetMatrixRva = 0x002B3890U;
constexpr std::uintptr_t kEntityNameResolverRva = 0x0018FE90U;
constexpr std::uintptr_t kNewtonTransformCallbackRva = 0x006DA960U;
constexpr std::uintptr_t kLuxPropOnUpdateRva = 0x0022B6A0U;
// cLuxAgent (AI characters): OnUpdate is vtable slot +0x120, and slot +0xF8
// is the mesh-entity getter, which returns the field at +0x940. The agent's
// transform comes from its character body at +0x948, not from prop nodes.
constexpr std::uintptr_t kLuxAgentOnUpdateRva = 0x000A6E00U;
constexpr std::uintptr_t kLuxAgentGetMeshRva = 0x0008FBC0U;
constexpr std::size_t kAgentMeshOffset = 0x940U;
constexpr std::size_t kVtableGetMeshSlot = 0xF8U;
constexpr std::size_t kVtableOnUpdateSlot = 0x120U;
constexpr std::size_t kEntityNameOffset = 0x100U;
constexpr std::size_t kEntityNodesBeginOffset = 0x8E8U;
constexpr std::size_t kEntityNodesEndOffset = 0x8F0U;
constexpr std::size_t kEntityFallbackNodeOffset = 0x9B0U;
constexpr std::size_t kEntityReferenceNodeOffset = 0x9B8U;
constexpr std::size_t kNodeLocalMatrixOffset = 0x48U;
constexpr std::size_t kMeshBonesBeginOffset = 0x260U;
constexpr std::size_t kMeshBonesEndOffset = 0x268U;
constexpr std::size_t kBoneLocalMatrixOffset = 0x3CU;
constexpr std::size_t kBoneParentOffset = 0x178U;
constexpr std::size_t kBoneAttachedNodesOffset = 0x188U;
constexpr std::size_t kBoneChildrenOffset = 0x198U;
constexpr std::size_t kStringSizeOffset = 0x10U;
constexpr std::size_t kStringCapacityOffset = 0x18U;
constexpr std::size_t kStringInlineCapacity = 15U;
constexpr std::size_t kMaximumNameLength = 96U;
constexpr std::size_t kMaximumNodes = 32U;
constexpr std::size_t kMaximumBones = pose::kMaximumBones;
constexpr std::size_t kMaximumHeldAttachments = 16U;
constexpr std::size_t kMaximumScriptedRoots = 256U;
constexpr std::size_t kMaximumInspectionRoots = 8U;
constexpr std::size_t kMaximumInspectionIdentities = 256U;
constexpr std::size_t kMaximumWorldBodyProbes = 128U;
constexpr std::size_t kMaximumInterpolatedWorldBodies = 256U;
constexpr std::size_t kMaximumEntityRootProbes = 128U;
constexpr std::size_t kMaximumArticulatedProps = 32U;
constexpr std::size_t kMinimumArticulatedPropBodies = 0U;
constexpr std::size_t kMaximumArticulatedPropBodies = 32U;
constexpr std::size_t kNewtonBodyPointerOffset = 0x2B0U;

constexpr std::array<std::uint8_t, 24> kEntitySetMatrixEntry{
    0x40, 0x53, 0x48, 0x81, 0xEC, 0xB0, 0x00, 0x00,
    0x00, 0x48, 0x8B, 0x01, 0x48, 0x8B, 0xD9, 0xC6,
    0x81, 0xCB, 0x03, 0x00, 0x00, 0x01, 0xFF, 0x90,
};
constexpr std::array<std::uint8_t, 24> kEntitySetVarBoolEntry{
    0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83,
    0xEC, 0x20, 0x41, 0x0F, 0xB6, 0xF8, 0x48, 0x8B,
    0xD9, 0xE8, 0x7A, 0xCA, 0xFE, 0xFF, 0x40, 0x84,
};
constexpr std::array<std::uint8_t, 24> kEntityGetMatrixEntry{
    0x40, 0x53, 0x48, 0x83, 0xEC, 0x20, 0x48, 0x8B,
    0x81, 0xF0, 0x08, 0x00, 0x00, 0x48, 0x8B, 0xDA,
    0x48, 0x2B, 0x81, 0xE8, 0x08, 0x00, 0x00, 0x48,
};
constexpr std::array<std::uint8_t, 24> kEntityVirtualSetMatrixEntry{
    0x48, 0x89, 0x5C, 0x24, 0x10, 0x57, 0x48, 0x81,
    0xEC, 0x20, 0x01, 0x00, 0x00, 0x48, 0x8B, 0x81,
    0xF0, 0x08, 0x00, 0x00, 0x48, 0x8B, 0xDA, 0x48,
};
constexpr std::array<std::uint8_t, 24> kMatrixInverseEntry{
    0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83,
    0xEC, 0x70, 0x48, 0x8B, 0xF9, 0x0F, 0x29, 0x74,
    0x24, 0x60, 0x48, 0x8B, 0xCA, 0x48, 0x8B, 0xDA,
};
constexpr std::array<std::uint8_t, 24> kMatrixMultiplyEntry{
    0x4C, 0x8B, 0xDC, 0x48, 0x81, 0xEC, 0xA8, 0x00,
    0x00, 0x00, 0xF3, 0x0F, 0x10, 0x6A, 0x04, 0xF3,
    0x0F, 0x10, 0x62, 0x08, 0xF3, 0x0F, 0x10, 0x1A,
};
constexpr std::array<std::uint8_t, 24> kNodeMatrixChangedEntry{
    0x48, 0x89, 0x5C, 0x24, 0x10, 0x48, 0x89, 0x6C,
    0x24, 0x18, 0x57, 0x48, 0x83, 0xEC, 0x60, 0xFF,
    0x81, 0xE0, 0x00, 0x00, 0x00, 0x48, 0x8B, 0xF9,
};
constexpr std::array<std::uint8_t, 24> kBoneSetMatrixEntry{
    0x0F, 0x10, 0x02, 0x0F, 0x11, 0x41, 0x3C, 0x0F,
    0x10, 0x4A, 0x10, 0x0F, 0x11, 0x49, 0x4C, 0x0F,
    0x10, 0x42, 0x20, 0x0F, 0x11, 0x41, 0x5C, 0x0F,
};
constexpr std::array<std::uint8_t, 24> kEntityNameResolverEntry{
    0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x6C,
    0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57,
    0x48, 0x83, 0xEC, 0x40, 0x48, 0x8B, 0x05, 0x8D,
};
constexpr std::array<std::uint8_t, 24> kNewtonTransformCallbackEntry{
    0x40, 0x53, 0x48, 0x83, 0xEC, 0x20, 0x48, 0x8B,
    0xDA, 0xFF, 0x15, 0xD9, 0xF0, 0x0B, 0x00, 0x8B,
    0x0B, 0x4C, 0x8B, 0xC0, 0x89, 0x48, 0x48, 0x8B,
};
constexpr std::array<std::uint8_t, 24> kLuxPropOnUpdateEntry{
    0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74,
    0x24, 0x10, 0x57, 0x48, 0x81, 0xEC, 0x80, 0x00,
    0x00, 0x00, 0x44, 0x0F, 0x29, 0x44, 0x24, 0x50,
};

constexpr std::array<std::uint8_t, 24> kLuxAgentOnUpdateEntry{
    0x40, 0x57, 0x48, 0x83, 0xEC, 0x40, 0xF3, 0x0F,
    0x58, 0x89, 0xF0, 0x03, 0x00, 0x00, 0xF3, 0x0F,
    0x10, 0x91, 0xEC, 0x03, 0x00, 0x00, 0x48, 0x8B,
};
constexpr std::array<std::uint8_t, 8> kLuxAgentGetMeshEntry{
    0x48, 0x8B, 0x81, 0x40, 0x09, 0x00, 0x00, 0xC3,
};

enum class Kind : std::uint32_t {
    other,
    hands,
    body,
    heldAttachment,
    scriptedRoot,
    scriptedPose,
    inspectionRoot,
};

using EntitySetMatrix = void(__fastcall*)(std::byte* entity,
                                           const root::Matrix* matrix);
using EntitySetVarBool = void(__fastcall*)(std::byte* entity,
                                            const std::byte* variableName,
                                            bool value);
using EntityGetMatrix = root::Matrix*(__fastcall*)(std::byte* entity,
                                                    root::Matrix* result);
using MatrixInverse = root::Matrix*(__fastcall*)(root::Matrix* result,
                                                 const root::Matrix* value);
using MatrixMultiply = root::Matrix*(__fastcall*)(root::Matrix* result,
                                                  const root::Matrix* left,
                                                  const root::Matrix* right);
using NodeMatrixChanged = void(__fastcall*)(std::byte* node, bool notify);
using BoneSetMatrix = void(__fastcall*)(std::byte* bone,
                                        const root::Matrix* matrix,
                                        bool notify);
using EntityNameResolver = std::byte**(__fastcall*)(
    const void* entityName, const void* apiName, std::uint32_t entityType);
using NewtonTransformCallback = void(__fastcall*)(
    const void* body, const float* matrix, int threadIndex);
using NewtonBodyGetUserData = void*(__cdecl*)(const void* body);
using LuxPropOnUpdate = void(__fastcall*)(std::byte* prop, float timeStep);

struct TrackedRoot {
    std::byte* entity = nullptr;
    root::Matrix staged{};
    bool hasStaged = false;
    root::History history{};
};

struct AppliedRoot {
    std::byte* entity = nullptr;
    root::Matrix authoritative{};
    std::array<std::byte*, kMaximumNodes> nodes{};
    std::array<root::Matrix, kMaximumNodes> matrices{};
    std::size_t count = 0;
    bool active = false;
};

struct TrackedNamedRoot {
    TrackedRoot root{};
    std::array<char, kMaximumNameLength + 1U> name{};
    std::uintptr_t callerRva = 0;
    std::uintptr_t vtableRva = 0;
};

struct InspectionIdentity {
    std::byte* entity = nullptr;
    std::array<char, kMaximumNameLength + 1U> name{};
};

struct BoneProbeState {
    std::byte* mesh = nullptr;
    std::size_t boneCount = 0;
    std::uint64_t lastHash = 0;
    bool hasHash = false;
    bool topologyLogged = false;
    std::uint64_t captures = 0;
    std::uint64_t changedCaptures = 0;
    std::uint64_t invalidCaptures = 0;
};

struct BoneCapture {
    std::byte* mesh = nullptr;
    std::array<std::byte*, kMaximumBones> bones{};
    std::array<root::Matrix, kMaximumBones> matrices{};
    std::size_t count = 0;
    bool changedFromPrevious = false;
};

struct TrackedPose {
    std::byte* mesh = nullptr;
    std::array<std::byte*, kMaximumBones> bones{};
    std::size_t count = 0;
    pose::History history{};
};

struct AppliedPose {
    std::array<std::byte*, kMaximumBones> bones{};
    std::array<root::Matrix, kMaximumBones> authoritative{};
    std::size_t count = 0;
    bool active = false;
};

struct ScriptedPoseEntry {
    const char* name = nullptr;
    const wchar_t* label = nullptr;
    std::byte* entity = nullptr;
    BoneProbeState probe{};
    TrackedPose pose{};
    AppliedPose applied{};
    bool registrationLogged = false;
    bool activationLogged = false;
    std::uint64_t lookupCalls = 0;
    std::uint64_t setMatrixCalls = 0;
    std::uint64_t published = 0;
    std::uint64_t publishResets = 0;
    std::uint64_t appliedCount = 0;
    std::uint64_t applyFailures = 0;
    std::uint64_t restoreFailures = 0;
};

struct WorldBodyProbe {
    std::byte* body = nullptr;
    std::array<char, kMaximumNameLength + 1U> name{};
    root::Matrix last{};
    bool hasLast = false;
    std::uint64_t callbacks = 0;
    std::uint64_t changedCallbacks = 0;
};

struct EntityRootProbe {
    std::byte* entity = nullptr;
    std::array<char, kMaximumNameLength + 1U> name{};
    std::uintptr_t callerRva = 0;
    std::uintptr_t vtableRva = 0;
    root::Matrix last{};
    bool hasLast = false;
    std::uint64_t callbacks = 0;
    std::uint64_t changedCallbacks = 0;
};

struct StagedWorldBody {
    std::byte* body = nullptr;
    const void* newtonBody = nullptr;
    std::array<char, kMaximumNameLength + 1U> name{};
    root::Matrix matrix{};
    bool available = false;
};

struct TrackedWorldBody {
    std::byte* body = nullptr;
    const void* newtonBody = nullptr;
    std::array<char, kMaximumNameLength + 1U> name{};
    std::uint64_t lastSeenSerial = 0;
    root::History history{};
};

struct AppliedWorldBody {
    std::byte* body = nullptr;
    const void* newtonBody = nullptr;
    std::array<char, kMaximumNameLength + 1U> name{};
    root::Matrix authoritative{};
    bool active = false;
};

struct ArticulatedPoseEntry {
    std::byte* entity = nullptr;
    std::array<char, kMaximumNameLength + 1U> name{};
    std::uint64_t lastSeenSerial = 0;
    std::size_t bodyCount = 0;
    // Where the owner keeps its mesh entity: props and agents differ.
    std::size_t meshOffset = kEntityFallbackNodeOffset;
    bool agent = false;
    // An agent moves its mesh itself every tick, so the mesh node needs its
    // own two-tick history; a prop's mesh follows already-handled roots.
    root::History meshRoot{};
    root::Matrix meshRootAuthoritative{};
    bool meshRootApplied = false;
    bool staged = false;
    bool interpolateCurrentInterval = false;
    bool activationLogged = false;
    BoneProbeState probe{};
    TrackedPose pose{};
    AppliedPose applied{};
    std::uint64_t published = 0;
    std::uint64_t publishResets = 0;
    std::uint64_t discontinuityResets = 0;
    std::uint64_t appliedCount = 0;
    std::uint64_t applyFailures = 0;
    std::uint64_t restoreFailures = 0;
    std::uint64_t unchangedRenderSkips = 0;
};

EntitySetMatrix g_originalEntitySetMatrix = nullptr;
EntitySetVarBool g_originalEntitySetVarBool = nullptr;
EntityNameResolver g_originalEntityNameResolver = nullptr;
NewtonTransformCallback g_originalNewtonTransformCallback = nullptr;
LuxPropOnUpdate g_originalLuxPropOnUpdate = nullptr;
LuxPropOnUpdate g_originalLuxAgentOnUpdate = nullptr;
NewtonBodyGetUserData g_newtonBodyGetUserData = nullptr;
EntityGetMatrix g_entityGetMatrix = nullptr;
MatrixInverse g_matrixInverse = nullptr;
MatrixMultiply g_matrixMultiply = nullptr;
NodeMatrixChanged g_nodeMatrixChanged = nullptr;
BoneSetMatrix g_boneSetMatrix = nullptr;
std::byte* g_executableBase = nullptr;
TrackedRoot g_hands{};
TrackedRoot g_body{};
AppliedRoot g_appliedHands{};
AppliedRoot g_appliedBody{};
std::array<TrackedRoot, kMaximumHeldAttachments> g_heldAttachments{};
std::array<AppliedRoot, kMaximumHeldAttachments> g_appliedHeldAttachments{};
std::array<TrackedNamedRoot, kMaximumScriptedRoots> g_scriptedRoots{};
std::array<AppliedRoot, kMaximumScriptedRoots> g_appliedScriptedRoots{};
std::array<TrackedNamedRoot, kMaximumInspectionRoots> g_inspectionRoots{};
std::array<AppliedRoot, kMaximumInspectionRoots> g_appliedInspectionRoots{};
std::array<InspectionIdentity, kMaximumInspectionIdentities>
    g_inspectionIdentities{};
BoneProbeState g_bodyBoneProbe{};
BoneProbeState g_handsBoneProbe{};
TrackedPose g_bodyPose{};
AppliedPose g_appliedBodyPose{};
// Scripted skeletal props are discovered structurally through
// cLuxProp::OnUpdate. No asset name is an interpolation allowlist.
std::array<ScriptedPoseEntry, 0> g_scriptedPoses{};
std::array<WorldBodyProbe, kMaximumWorldBodyProbes> g_worldBodyProbes{};
std::array<StagedWorldBody, kMaximumInterpolatedWorldBodies>
    g_stagedWorldBodies{};
std::array<TrackedWorldBody, kMaximumInterpolatedWorldBodies>
    g_worldBodies{};
std::array<AppliedWorldBody, kMaximumInterpolatedWorldBodies>
    g_appliedWorldBodies{};
std::array<ArticulatedPoseEntry, kMaximumArticulatedProps>
    g_articulatedPoses{};
std::atomic_flag g_worldBodyProbeLock = ATOMIC_FLAG_INIT;
std::atomic_flag g_worldStageLock = ATOMIC_FLAG_INIT;
std::atomic<std::uint64_t> g_worldBodyProbeLockDrops{0};
std::atomic<std::uint64_t> g_worldBodyProbeInvalid{0};
std::atomic<std::uint64_t> g_worldBodyProbeOverflows{0};
std::atomic<std::uint64_t> g_worldStageOverflows{0};
std::array<EntityRootProbe, kMaximumEntityRootProbes> g_entityRootProbes{};
std::atomic_flag g_entityRootProbeLock = ATOMIC_FLAG_INIT;
std::atomic<std::uint64_t> g_entityRootProbeLockDrops{0};
std::atomic<std::uint64_t> g_entityRootProbeInvalid{0};
std::atomic<std::uint64_t> g_entityRootProbeOverflows{0};
bool g_interpolationEnabled = false;
bool g_poseInterpolationEnabled = false;
bool g_worldInterpolationEnabled = false;
bool g_articulatedPoseInterpolationEnabled = false;
bool g_diagnosticsEnabled = false;
bool g_renderActive = false;
std::atomic<bool> g_loggedHands{false};
std::atomic<bool> g_loggedBody{false};
std::atomic<bool> g_loggedHeldAttachment{false};
std::atomic<bool> g_loggedScriptedRoot{false};
std::atomic<bool> g_loggedInterpolation{false};
std::atomic<bool> g_loggedPoseInterpolation{false};
std::atomic<bool> g_loggedHeldInterpolation{false};
std::atomic<bool> g_loggedScriptedRootInterpolation{false};
std::atomic<bool> g_loggedInspectionInterpolation{false};
std::atomic<bool> g_loggedWorldInterpolation{false};
std::atomic<bool> g_loggedArticulatedPoseInterpolation{false};
std::uint64_t g_handsCalls = 0;
std::uint64_t g_bodyCalls = 0;
std::uint64_t g_heldAttachmentCalls = 0;
std::uint64_t g_scriptedRootCalls = 0;
std::uint64_t g_invalidMatrices = 0;
std::uint64_t g_rootApplied = 0;
std::uint64_t g_rootApplyFailures = 0;
std::uint64_t g_rootRestoreFailures = 0;
std::uint64_t g_readRootFailures = 0;
std::uint64_t g_collectNodeFailures = 0;
std::uint64_t g_inverseFailures = 0;
std::uint64_t g_multiplyFailures = 0;
std::uint64_t g_verifyFailures = 0;
std::uint64_t g_posePublished = 0;
std::uint64_t g_posePublishResets = 0;
std::uint64_t g_poseApplied = 0;
std::uint64_t g_poseApplyFailures = 0;
std::uint64_t g_poseRestoreFailures = 0;
std::uint64_t g_heldApplied = 0;
std::uint64_t g_heldApplyFailures = 0;
std::uint64_t g_heldRestoreFailures = 0;
std::uint64_t g_heldRegistryOverflows = 0;
std::uint64_t g_scriptedRootApplied = 0;
std::uint64_t g_scriptedRootApplyFailures = 0;
std::uint64_t g_scriptedRootRestoreFailures = 0;
std::uint64_t g_scriptedRootRegistryOverflows = 0;
std::uint64_t g_inspectionRootCalls = 0;
std::uint64_t g_inspectionApplied = 0;
std::uint64_t g_inspectionApplyFailures = 0;
std::uint64_t g_inspectionRestoreFailures = 0;
std::uint64_t g_inspectionRegistryOverflows = 0;
std::uint64_t g_worldPublished = 0;
std::uint64_t g_worldPublishResets = 0;
std::uint64_t g_worldApplied = 0;
std::uint64_t g_worldApplyFailures = 0;
std::uint64_t g_worldRestoreFailures = 0;
std::uint64_t g_articulatedRegistryOverflows = 0;
LARGE_INTEGER g_performanceFrequency{};
std::uint64_t g_articulatedCaptureTicks = 0;
std::uint64_t g_articulatedCaptureCalls = 0;
std::uint64_t g_renderApplyTicks = 0;
std::uint64_t g_renderApplyCalls = 0;
std::uint64_t g_renderRestoreTicks = 0;
std::uint64_t g_renderRestoreCalls = 0;
std::uint64_t g_articulatedBonesWritten = 0;
// Parent index of each PlayerBody bone within the captured bone vector, and
// the camera-attachment state for the tick pair currently being presented.
std::array<std::int16_t, kMaximumBones> g_bodyParents{};
bool g_bodyParentsValid = false;
attach::Attachment g_attachment{};
std::uint64_t g_attachmentSerial = 0;
std::uint64_t g_attachmentGeneration = 0;
std::uint64_t g_attachmentFrames = 0;
std::uint64_t g_attachmentSearches = 0;
std::uint64_t g_attachmentFound = 0;
std::uint64_t g_attachmentToggles = 0;
std::uint64_t g_attachmentBoneChanges = 0;
std::uint64_t g_attachmentApplied = 0;
float g_attachmentWorstError = 0.0F;
constexpr float kAttachmentTolerance = 2.0e-3F;
std::uint64_t g_agentRootApplied = 0;
std::uint64_t g_agentRootPublished = 0;
std::uint64_t g_agentRootResets = 0;
root::Matrix g_viewYawLead{};
bool g_viewYawLeadActive = false;
std::array<root::Matrix, kMaximumBones> g_previousBodyBones{};
std::byte* g_previousBodyMesh = nullptr;
std::size_t g_previousBodyBoneCount = 0;
float g_bodyMaximumPositionDelta = 0.0F;
float g_bodyMaximumOrientationDelta = 0.0F;
std::uint64_t g_bodyTicksOverPositionLimit = 0;
std::uint64_t g_bodyTicksOverOrientationLimit = 0;
ULONGLONG g_lastReportMilliseconds = 0;

constexpr std::uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

std::int64_t PerformanceCounterNow() noexcept {
    if (!g_diagnosticsEnabled || g_performanceFrequency.QuadPart <= 0) {
        return 0;
    }
    LARGE_INTEGER counter{};
    return QueryPerformanceCounter(&counter) ? counter.QuadPart : 0;
}

void AccumulatePerformance(const std::int64_t started,
                           std::uint64_t& elapsed,
                           std::uint64_t& calls) noexcept {
    if (started <= 0) {
        return;
    }
    const std::int64_t finished = PerformanceCounterNow();
    if (finished >= started) {
        elapsed += static_cast<std::uint64_t>(finished - started);
        ++calls;
    }
}

double PerformanceMilliseconds(const std::uint64_t ticks) noexcept {
    return g_performanceFrequency.QuadPart <= 0
        ? 0.0
        : static_cast<double>(ticks) * 1000.0 /
              static_cast<double>(g_performanceFrequency.QuadPart);
}

double PerformanceMicrosecondsPerCall(const std::uint64_t ticks,
                                      const std::uint64_t calls) noexcept {
    return calls == 0U ? 0.0 : PerformanceMilliseconds(ticks) * 1000.0 /
        static_cast<double>(calls);
}

// VirtualQuery is a system call, and one render scope validates thousands of
// bone and node addresses that live in a handful of heap regions. Each thread
// therefore remembers the committed regions it has already queried. The main
// thread advances the epoch at every tick commit and render begin/end, so a
// remembered region is never trusted beyond the phase that queried it.
struct AccessRegion {
    std::uintptr_t begin = 0;
    std::uintptr_t end = 0;
    bool writable = false;
};

struct AccessRegionCache {
    std::array<AccessRegion, 32> regions{};
    std::uint32_t epoch = 0;
    std::size_t lastHit = 0;
    std::size_t next = 0;
};

std::atomic<std::uint32_t> g_accessEpoch{1};
std::atomic<std::uint64_t> g_accessQueries{0};
thread_local AccessRegionCache t_accessRegions;

void InvalidateAccessRegions() noexcept {
    g_accessEpoch.fetch_add(1U, std::memory_order_relaxed);
}

bool IsAccessible(const void* const address, const std::size_t bytes,
                  const bool writable) noexcept {
    if (address == nullptr || bytes == 0U) {
        return false;
    }
    const auto start = reinterpret_cast<std::uintptr_t>(address);
    if (bytes > UINTPTR_MAX - start) {
        return false;
    }
    const std::uintptr_t finish = start + bytes;

    AccessRegionCache& cache = t_accessRegions;
    const std::uint32_t epoch = g_accessEpoch.load(std::memory_order_relaxed);
    if (cache.epoch != epoch) {
        cache.regions = {};
        cache.lastHit = 0;
        cache.next = 0;
        cache.epoch = epoch;
    }
    const auto covers = [start, finish](const AccessRegion& region) noexcept {
        return start >= region.begin && finish <= region.end;
    };
    if (covers(cache.regions[cache.lastHit])) {
        return cache.regions[cache.lastHit].writable || !writable;
    }
    for (std::size_t index = 0; index < cache.regions.size(); ++index) {
        if (covers(cache.regions[index])) {
            cache.lastHit = index;
            return cache.regions[index].writable || !writable;
        }
    }

    g_accessQueries.fetch_add(1U, std::memory_order_relaxed);
    MEMORY_BASIC_INFORMATION memory{};
    if (VirtualQuery(address, &memory, sizeof(memory)) != sizeof(memory) ||
        memory.State != MEM_COMMIT ||
        (memory.Protect & (PAGE_NOACCESS | PAGE_GUARD)) != 0U) {
        return false;
    }
    constexpr DWORD readable = PAGE_READONLY | PAGE_READWRITE | PAGE_WRITECOPY |
        PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
    constexpr DWORD writeable = PAGE_READWRITE | PAGE_WRITECOPY |
        PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY;
    if ((memory.Protect & readable) == 0U) {
        return false;
    }
    const bool regionWritable = (memory.Protect & writeable) != 0U;
    const auto regionStart = reinterpret_cast<std::uintptr_t>(memory.BaseAddress);
    const auto regionEnd = regionStart + memory.RegionSize;

    // A query that starts lower in an already remembered region reports the
    // same end; widen that entry instead of spending another slot on it.
    std::size_t slot = cache.next;
    for (std::size_t index = 0; index < cache.regions.size(); ++index) {
        if (cache.regions[index].end == regionEnd) {
            slot = index;
            break;
        }
    }
    if (slot == cache.next) {
        cache.next = (cache.next + 1U) % cache.regions.size();
    }
    cache.regions[slot] = {regionStart, regionEnd, regionWritable};
    cache.lastHit = slot;

    return (regionWritable || !writable) && start >= regionStart &&
           finish <= regionEnd;
}

template <typename T>
bool Read(const void* const address, T& value) noexcept {
    if (!IsAccessible(address, sizeof(T), false)) {
        return false;
    }
    std::memcpy(&value, address, sizeof(T));
    return true;
}

template <std::size_t Size>
bool Matches(const std::byte* const address,
             const std::array<std::uint8_t, Size>& signature) noexcept {
    return IsAccessible(address, Size, false) &&
           std::memcmp(address, signature.data(), Size) == 0;
}

std::wstring Hex(const std::uintptr_t value) {
    wchar_t buffer[32]{};
    _snwprintf_s(buffer, std::size(buffer), _TRUNCATE, L"0x%llX",
                 static_cast<unsigned long long>(value));
    return buffer;
}

bool ReadStringObject(const std::byte* const stringObject,
                      std::string& value) noexcept {
    value.clear();
    if (stringObject == nullptr) {
        return false;
    }
    std::size_t length = 0;
    std::size_t capacity = 0;
    if (!Read(stringObject + kStringSizeOffset, length) ||
        !Read(stringObject + kStringCapacityOffset, capacity) ||
        length == 0U || length > kMaximumNameLength || capacity < length) {
        return false;
    }
    const char* characters = reinterpret_cast<const char*>(stringObject);
    if (capacity > kStringInlineCapacity && !Read(stringObject, characters)) {
        return false;
    }
    if (!IsAccessible(characters, length, false)) {
        return false;
    }
    value.assign(characters, length);
    return true;
}

Kind ReadKind(const std::byte* const entity, std::string& name) noexcept {
    name.clear();
    if (entity == nullptr) {
        return Kind::other;
    }
    const std::byte* const stringObject = entity + kEntityNameOffset;
    if (!ReadStringObject(stringObject, name)) {
        return Kind::other;
    }
    if (name.starts_with("PlayerHands_")) {
        return Kind::hands;
    }
    if (name.starts_with("PlayerBody_")) {
        return Kind::body;
    }
    if (name.ends_with("_Held") || name.ends_with("_Accessory")) {
        return Kind::heldAttachment;
    }
    for (const InspectionIdentity& identity : g_inspectionIdentities) {
        if (identity.entity == entity && name == identity.name.data()) {
            return Kind::inspectionRoot;
        }
    }
    for (const TrackedNamedRoot& entry : g_scriptedRoots) {
        if (entry.root.entity == entity && name == entry.name.data()) {
            return Kind::scriptedRoot;
        }
    }
    return Kind::other;
}

void ObserveWorldBody(std::byte* const body,
                      const void* const newtonBody) noexcept {
    if (body == nullptr || newtonBody == nullptr) {
        ++g_worldBodyProbeInvalid;
        return;
    }

    std::string name;
    root::Matrix matrix{};
    root::TransformInfo info{};
    if (!ReadStringObject(body + 0x10U, name) ||
        !Read(body + kNodeLocalMatrixOffset, matrix) ||
        !root::Analyze(matrix, info)) {
        ++g_worldBodyProbeInvalid;
        return;
    }

    if (g_worldInterpolationEnabled) {
        while (g_worldStageLock.test_and_set(std::memory_order_acquire)) {
            SwitchToThread();
        }
        StagedWorldBody* staged = nullptr;
        StagedWorldBody* freeSlot = nullptr;
        for (StagedWorldBody& candidate : g_stagedWorldBodies) {
            if (candidate.available && candidate.body == body &&
                candidate.newtonBody == newtonBody) {
                staged = &candidate;
                break;
            }
            if (!candidate.available && freeSlot == nullptr) {
                freeSlot = &candidate;
            }
        }
        staged = staged != nullptr ? staged : freeSlot;
        if (staged != nullptr) {
            *staged = {};
            staged->body = body;
            staged->newtonBody = newtonBody;
            const std::size_t copied =
                std::min(name.size(), staged->name.size() - 1U);
            std::memcpy(staged->name.data(), name.data(), copied);
            staged->name[copied] = '\0';
            staged->matrix = matrix;
            staged->available = true;
        } else {
            ++g_worldStageOverflows;
        }
        g_worldStageLock.clear(std::memory_order_release);
    }

    if (g_worldBodyProbeLock.test_and_set(std::memory_order_acquire)) {
        ++g_worldBodyProbeLockDrops;
        return;
    }

    if (!g_diagnosticsEnabled) {
        g_worldBodyProbeLock.clear(std::memory_order_release);
        return;
    }

    WorldBodyProbe* available = nullptr;
    for (WorldBodyProbe& probe : g_worldBodyProbes) {
        if (probe.body == body) {
            if (name != probe.name.data()) {
                probe = {};
                probe.body = body;
            }
            available = &probe;
            break;
        }
        if (available == nullptr && probe.body == nullptr) {
            available = &probe;
        }
    }
    if (available == nullptr) {
        g_worldBodyProbeLock.clear(std::memory_order_release);
        ++g_worldBodyProbeOverflows;
        return;
    }
    if (available->body == nullptr) {
        available->body = body;
        const std::size_t copied =
            std::min(name.size(), available->name.size() - 1U);
        std::memcpy(available->name.data(), name.data(), copied);
        available->name[copied] = '\0';
    }
    ++available->callbacks;
    if (available->hasLast && !root::NearlyEqual(available->last, matrix,
                                                  1.0e-6F)) {
        ++available->changedCallbacks;
    }
    available->last = matrix;
    available->hasLast = true;
    g_worldBodyProbeLock.clear(std::memory_order_release);
}

void ObserveEntityRoot(std::byte* const entity,
                       const std::string& name,
                       const root::Matrix* const matrix,
                       const std::uintptr_t callerRva,
                       const std::uintptr_t vtableRva) noexcept {
    root::Matrix copy{};
    root::TransformInfo info{};
    if (entity == nullptr || name.empty() || !Read(matrix, copy) ||
        !root::Analyze(copy, info)) {
        ++g_entityRootProbeInvalid;
        return;
    }

    if (g_entityRootProbeLock.test_and_set(std::memory_order_acquire)) {
        ++g_entityRootProbeLockDrops;
        return;
    }

    EntityRootProbe* available = nullptr;
    for (EntityRootProbe& probe : g_entityRootProbes) {
        if (probe.entity == entity && probe.callerRva == callerRva) {
            if (name != probe.name.data()) {
                probe = {};
                probe.entity = entity;
                probe.callerRva = callerRva;
                probe.vtableRva = vtableRva;
            }
            available = &probe;
            break;
        }
        if (available == nullptr && probe.entity == nullptr) {
            available = &probe;
        }
    }
    if (available == nullptr) {
        g_entityRootProbeLock.clear(std::memory_order_release);
        ++g_entityRootProbeOverflows;
        return;
    }
    if (available->entity == nullptr) {
        available->entity = entity;
        available->callerRva = callerRva;
        available->vtableRva = vtableRva;
        const std::size_t copied =
            std::min(name.size(), available->name.size() - 1U);
        std::memcpy(available->name.data(), name.data(), copied);
        available->name[copied] = '\0';
    }
    ++available->callbacks;
    if (available->hasLast &&
        !root::NearlyEqual(available->last, copy, 1.0e-6F)) {
        ++available->changedCallbacks;
    }
    available->last = copy;
    available->hasLast = true;
    g_entityRootProbeLock.clear(std::memory_order_release);
}

ScriptedPoseEntry* FindScriptedPose(const std::string& name) noexcept {
    for (ScriptedPoseEntry& entry : g_scriptedPoses) {
        if (entry.name != nullptr && name == entry.name) {
            return &entry;
        }
    }
    return nullptr;
}

std::uintptr_t FunctionRva(const void* const function) noexcept {
    const auto address = reinterpret_cast<std::uintptr_t>(function);
    const auto base = reinterpret_cast<std::uintptr_t>(g_executableBase);
    return address >= base ? address - base : address;
}

bool ValidEntityVtable(std::byte* const entity) noexcept {
    void** vtable = nullptr;
    void* getter = nullptr;
    void* setter = nullptr;
    return Read(entity, vtable) && IsAccessible(vtable, 0x168U, false) &&
           Read(vtable + (0xA0U / sizeof(void*)), getter) &&
           Read(vtable + (0x160U / sizeof(void*)), setter) &&
           getter == reinterpret_cast<void*>(g_entityGetMatrix) &&
           setter == g_executableBase + kEntityVirtualSetMatrixRva;
}

void RegisterInspectionIdentity(std::byte* const entity) noexcept {
    std::string name;
    if (!ReadStringObject(entity + kEntityNameOffset, name) ||
        !ValidEntityVtable(entity)) {
        ++g_inspectionRegistryOverflows;
        return;
    }

    InspectionIdentity* available = nullptr;
    for (InspectionIdentity& identity : g_inspectionIdentities) {
        if (identity.entity == entity) {
            if (name == identity.name.data()) {
                return;
            }
            identity = {};
            available = &identity;
            break;
        }
        if (available == nullptr && identity.entity == nullptr) {
            available = &identity;
            continue;
        }
        if (available == nullptr) {
            std::string currentName;
            if (!ReadStringObject(identity.entity + kEntityNameOffset,
                                  currentName) ||
                currentName != identity.name.data()) {
                identity = {};
                available = &identity;
            }
        }
    }
    if (available == nullptr) {
        ++g_inspectionRegistryOverflows;
        return;
    }

    available->entity = entity;
    const std::size_t copied =
        std::min(name.size(), available->name.size() - 1U);
    std::memcpy(available->name.data(), name.data(), copied);
    available->name[copied] = '\0';
    std::wstring wideName(name.begin(), name.end());
    log::Write(log::Level::info,
               L"Readable inspection entity registered by IsHUDVersion: " +
                   wideName + L".");
}

void Reset(TrackedRoot& tracked) noexcept {
    tracked.entity = nullptr;
    tracked.hasStaged = false;
    tracked.history.Reset();
}

void Stage(TrackedRoot& tracked, std::byte* const entity,
           const root::Matrix& matrix) noexcept {
    if (tracked.entity != entity) {
        Reset(tracked);
        tracked.entity = entity;
    }
    tracked.staged = matrix;
    tracked.hasStaged = true;
}

TrackedRoot* FindHeldAttachment(std::byte* const entity) noexcept {
    for (TrackedRoot& tracked : g_heldAttachments) {
        if (tracked.entity == entity) {
            return &tracked;
        }
    }
    return nullptr;
}

void ResetHeldAttachment(std::byte* const entity) noexcept {
    if (TrackedRoot* const tracked = FindHeldAttachment(entity);
        tracked != nullptr) {
        Reset(*tracked);
    }
}

void StageHeldAttachment(std::byte* const entity, const root::Matrix& matrix,
                         const std::string& name) noexcept {
    if (TrackedRoot* const existing = FindHeldAttachment(entity);
        existing != nullptr) {
        Stage(*existing, entity, matrix);
        return;
    }

    TrackedRoot* available = nullptr;
    for (TrackedRoot& tracked : g_heldAttachments) {
        if (tracked.entity == nullptr) {
            available = &tracked;
            break;
        }
        std::string previousName;
        if (ReadKind(tracked.entity, previousName) != Kind::heldAttachment) {
            Reset(tracked);
            available = &tracked;
            break;
        }
    }
    if (available == nullptr) {
        ++g_heldRegistryOverflows;
        return;
    }
    Stage(*available, entity, matrix);
    std::wstring wideName(name.begin(), name.end());
    log::Write(log::Level::info,
               L"Held/accessory root registered for interpolation: " +
                   wideName + L".");
}

TrackedNamedRoot* FindScriptedRoot(std::byte* const entity) noexcept {
    for (TrackedNamedRoot& entry : g_scriptedRoots) {
        if (entry.root.entity == entity) {
            return &entry;
        }
    }
    return nullptr;
}

void ResetScriptedRoot(TrackedNamedRoot& entry) noexcept {
    Reset(entry.root);
    entry.name = {};
    entry.callerRva = 0;
    entry.vtableRva = 0;
}

void ResetScriptedRoot(std::byte* const entity) noexcept {
    if (TrackedNamedRoot* const entry = FindScriptedRoot(entity);
        entry != nullptr) {
        ResetScriptedRoot(*entry);
    }
}

void StageScriptedRoot(std::byte* const entity, const root::Matrix& matrix,
                       const std::string& name,
                       const std::uintptr_t callerRva,
                       const std::uintptr_t vtableRva) noexcept {
    if (TrackedNamedRoot* const existing = FindScriptedRoot(entity);
        existing != nullptr) {
        if (name == existing->name.data() &&
            vtableRva == existing->vtableRva) {
            Stage(existing->root, entity, matrix);
            existing->callerRva = callerRva;
            return;
        }
        ResetScriptedRoot(*existing);
    }

    TrackedNamedRoot* available = nullptr;
    for (TrackedNamedRoot& entry : g_scriptedRoots) {
        if (entry.root.entity == nullptr) {
            available = &entry;
            break;
        }
        std::string previousName;
        void** previousVtable = nullptr;
        const bool validName = ReadStringObject(
            entry.root.entity + kEntityNameOffset, previousName);
        const std::uintptr_t previousVtableRva =
            Read(entry.root.entity, previousVtable)
            ? FunctionRva(previousVtable)
            : 0U;
        if (!validName || previousName != entry.name.data() ||
            previousVtableRva != entry.vtableRva) {
            ResetScriptedRoot(entry);
            available = &entry;
            break;
        }
    }
    if (available == nullptr) {
        ++g_scriptedRootRegistryOverflows;
        return;
    }
    Stage(available->root, entity, matrix);
    const std::size_t copied =
        std::min(name.size(), available->name.size() - 1U);
    std::memcpy(available->name.data(), name.data(), copied);
    available->name[copied] = '\0';
    available->callerRva = callerRva;
    available->vtableRva = vtableRva;
}

void ResetInspectionRoot(TrackedNamedRoot& entry) noexcept {
    Reset(entry.root);
    entry.name = {};
    entry.callerRva = 0;
    entry.vtableRva = 0;
}

TrackedNamedRoot* FindInspectionRoot(std::byte* const entity) noexcept {
    for (TrackedNamedRoot& entry : g_inspectionRoots) {
        if (entry.root.entity == entity) {
            return &entry;
        }
    }
    return nullptr;
}

void ResetInspectionRoot(std::byte* const entity) noexcept {
    if (TrackedNamedRoot* const entry = FindInspectionRoot(entity);
        entry != nullptr) {
        ResetInspectionRoot(*entry);
    }
}

void StageInspectionRoot(std::byte* const entity,
                         const root::Matrix& matrix,
                         const std::string& name,
                         const std::uintptr_t callerRva,
                         const std::uintptr_t vtableRva) noexcept {
    if (TrackedNamedRoot* const existing = FindInspectionRoot(entity);
        existing != nullptr) {
        if (name != existing->name.data()) {
            ResetInspectionRoot(*existing);
            existing->root.entity = entity;
            const std::size_t copied =
                std::min(name.size(), existing->name.size() - 1U);
            std::memcpy(existing->name.data(), name.data(), copied);
            existing->name[copied] = '\0';
        }
        existing->callerRva = callerRva;
        existing->vtableRva = vtableRva;
        Stage(existing->root, entity, matrix);
        return;
    }

    TrackedNamedRoot* available = nullptr;
    for (TrackedNamedRoot& entry : g_inspectionRoots) {
        if (entry.root.entity == nullptr) {
            available = &entry;
            break;
        }
        std::string previousName;
        if (ReadKind(entry.root.entity, previousName) !=
                Kind::inspectionRoot ||
            previousName != entry.name.data()) {
            ResetInspectionRoot(entry);
            available = &entry;
            break;
        }
    }
    if (available == nullptr) {
        // The readable state presents only one open prop at a time. Retired
        // roots may retain a valid semantic identity for later reopening, so
        // recycle their interpolation state without discarding that identity.
        for (TrackedNamedRoot& entry : g_inspectionRoots) {
            if (entry.root.entity != entity) {
                ResetInspectionRoot(entry);
                available = &entry;
                break;
            }
        }
        if (available == nullptr) {
            ++g_inspectionRegistryOverflows;
            return;
        }
    }
    const std::size_t copied =
        std::min(name.size(), available->name.size() - 1U);
    std::memcpy(available->name.data(), name.data(), copied);
    available->name[copied] = '\0';
    available->callerRva = callerRva;
    available->vtableRva = vtableRva;
    Stage(available->root, entity, matrix);
    std::wstring wideName(name.begin(), name.end());
    log::Write(log::Level::info,
               L"Inspection root registered for interpolation: " +
                   wideName + L"; caller/vtable RVA=" + Hex(callerRva) +
                   L"/" + Hex(vtableRva) + L".");
}

void LogDiscovery(const Kind kind, const std::string& name,
                  std::byte* const entity, const root::Matrix& matrix,
                  const root::TransformInfo& info) {
    std::atomic<bool>& flag = kind == Kind::hands
        ? g_loggedHands
        : (kind == Kind::body
               ? g_loggedBody
               : (kind == Kind::scriptedRoot ? g_loggedScriptedRoot
                                              : g_loggedHeldAttachment));
    if (flag.exchange(true, std::memory_order_acq_rel)) {
        return;
    }
    void** vtable = nullptr;
    void* getter = nullptr;
    void* setter = nullptr;
    if (Read(entity, vtable) && IsAccessible(vtable, 0x168U, false)) {
        static_cast<void>(Read(vtable + (0xA0U / sizeof(void*)), getter));
        static_cast<void>(Read(vtable + (0x160U / sizeof(void*)), setter));
    }
    std::byte** nodesBegin = nullptr;
    std::byte** nodesEnd = nullptr;
    std::byte* referenceNode = nullptr;
    std::size_t nodeCount = 0;
    long long referenceIndex = -1;
    std::byte* fallbackNode = nullptr;
    static_cast<void>(Read(entity + kEntityFallbackNodeOffset, fallbackNode));
    if (Read(entity + kEntityNodesBeginOffset, nodesBegin) &&
        Read(entity + kEntityNodesEndOffset, nodesEnd) &&
        nodesBegin != nullptr && nodesEnd >= nodesBegin) {
        const auto nodeBytes = reinterpret_cast<std::uintptr_t>(nodesEnd) -
                               reinterpret_cast<std::uintptr_t>(nodesBegin);
        if (nodeBytes % sizeof(std::byte*) == 0U &&
            IsAccessible(nodesBegin, nodeBytes, false)) {
            nodeCount = nodeBytes / sizeof(std::byte*);
            static_cast<void>(
                Read(entity + kEntityReferenceNodeOffset, referenceNode));
            if (referenceNode == nullptr && nodeCount != 0U) {
                referenceNode = nodesBegin[0];
            }
            for (std::size_t index = 0; index < nodeCount; ++index) {
                if (nodesBegin[index] == referenceNode) {
                    referenceIndex = static_cast<long long>(index);
                    break;
                }
            }
        }
    }
    root::Matrix getterMatrix{};
    root::TransformInfo getterInfo{};
    float getterDifference = -1.0F;
    if (g_entityGetMatrix != nullptr &&
        g_entityGetMatrix(entity, &getterMatrix) == &getterMatrix &&
        root::Analyze(getterMatrix, getterInfo)) {
        getterDifference = 0.0F;
        for (std::size_t index = 0; index < matrix.values.size(); ++index) {
            getterDifference =
                std::max(getterDifference,
                         std::abs(getterMatrix.values[index] -
                                  matrix.values[index]));
        }
    }
    std::wstring wideName(name.begin(), name.end());
    log::Write(log::Level::info,
               L"Root probe found " + wideName + L": uniform-scale=" +
                   std::to_wstring(info.uniformScale) + L", translation=(" +
                   std::to_wstring(matrix.values[3]) + L", " +
                   std::to_wstring(matrix.values[7]) + L", " +
                   std::to_wstring(matrix.values[11]) +
                   L"), virtual getter/setter RVAs=" + Hex(FunctionRva(getter)) +
                   L"/" + Hex(FunctionRva(setter)) + L", nodes=" +
                   std::to_wstring(nodeCount) + L", reference-index=" +
                   std::to_wstring(referenceIndex) +
                   L", fallback-node=" +
                   std::wstring(fallbackNode == nullptr ? L"false" : L"true") +
                   L", setter/getter max-delta=" +
                   std::to_wstring(getterDifference) + L".");
}

bool IsFiniteAffine(const root::Matrix& matrix) noexcept {
    for (const float value : matrix.values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    constexpr float tolerance = 1.0e-4F;
    return std::abs(matrix.values[12]) <= tolerance &&
           std::abs(matrix.values[13]) <= tolerance &&
           std::abs(matrix.values[14]) <= tolerance &&
           std::abs(matrix.values[15] - 1.0F) <= tolerance;
}

bool SameMatrixBits(const root::Matrix& left,
                    const root::Matrix& right) noexcept {
    return std::memcmp(left.values.data(), right.values.data(),
                       sizeof(left.values)) == 0;
}

void HashBytes(std::uint64_t& hash, const void* const data,
               const std::size_t size) noexcept {
    const auto* const bytes = static_cast<const std::uint8_t*>(data);
    for (std::size_t index = 0; index < size; ++index) {
        hash ^= bytes[index];
        hash *= kFnvPrime;
    }
}

bool CaptureBones(std::byte* const entity, BoneProbeState& probe,
                  const wchar_t* const label, BoneCapture& capture,
                  const std::size_t meshOffset =
                      kEntityFallbackNodeOffset) noexcept {
    capture = {};
    if ((!g_diagnosticsEnabled && !g_poseInterpolationEnabled &&
         !g_articulatedPoseInterpolationEnabled) ||
        entity == nullptr) {
        return false;
    }
    const auto invalid = [&probe]() noexcept {
        if (g_diagnosticsEnabled) {
            ++probe.invalidCaptures;
        }
        return false;
    };

    std::byte** begin = nullptr;
    std::byte** end = nullptr;
    if (!Read(entity + meshOffset, capture.mesh) ||
        capture.mesh == nullptr ||
        !Read(capture.mesh + kMeshBonesBeginOffset, begin) ||
        !Read(capture.mesh + kMeshBonesEndOffset, end) || begin == nullptr ||
        end < begin) {
        return invalid();
    }

    const auto byteCount = reinterpret_cast<std::uintptr_t>(end) -
                           reinterpret_cast<std::uintptr_t>(begin);
    if (byteCount % sizeof(std::byte*) != 0U) {
        return invalid();
    }
    capture.count = byteCount / sizeof(std::byte*);
    if (capture.count == 0U || capture.count > kMaximumBones ||
        !IsAccessible(begin, byteCount, false)) {
        return invalid();
    }

    std::memcpy(capture.bones.data(), begin, byteCount);
    std::size_t affineCount = 0;
    std::size_t uniformRigidCount = 0;
    std::uint64_t hash = kFnvOffsetBasis;
    HashBytes(hash, &capture.count, sizeof(capture.count));
    for (std::size_t index = 0; index < capture.count; ++index) {
        if (capture.bones[index] == nullptr ||
            !Read(capture.bones[index] + kBoneLocalMatrixOffset,
                  capture.matrices[index])) {
            return invalid();
        }
        if (IsFiniteAffine(capture.matrices[index])) {
            ++affineCount;
        }
        root::TransformInfo info{};
        if (root::Analyze(capture.matrices[index], info)) {
            ++uniformRigidCount;
        }
        HashBytes(hash, capture.matrices[index].values.data(),
                  capture.matrices[index].values.size() * sizeof(float));
    }

    if (probe.mesh != capture.mesh || probe.boneCount != capture.count) {
        probe.mesh = capture.mesh;
        probe.boneCount = capture.count;
        probe.lastHash = 0;
        probe.hasHash = false;
        probe.topologyLogged = false;
    }

    capture.changedFromPrevious = probe.hasHash && probe.lastHash != hash;
    if (g_diagnosticsEnabled) {
        ++probe.captures;
        if (capture.changedFromPrevious) {
            ++probe.changedCaptures;
        }
    }
    probe.lastHash = hash;
    probe.hasHash = true;

    if (!g_diagnosticsEnabled) {
        return true;
    }

    if (!probe.topologyLogged) {
        std::size_t parentLinks = 0;
        std::size_t knownParents = 0;
        std::size_t earlierParents = 0;
        std::size_t bonesWithAttachedNodes = 0;
        std::size_t bonesWithChildren = 0;
        std::size_t invalidLists = 0;
        for (std::size_t index = 0; index < capture.count; ++index) {
            for (const auto [offset, count] :
                 std::array<std::pair<std::size_t, std::size_t*>, 2>{
                     std::pair{kBoneAttachedNodesOffset,
                               &bonesWithAttachedNodes},
                     std::pair{kBoneChildrenOffset, &bonesWithChildren}}) {
                std::byte* sentinel = nullptr;
                std::byte* first = nullptr;
                if (!Read(capture.bones[index] + offset, sentinel) ||
                    sentinel == nullptr || !Read(sentinel, first)) {
                    ++invalidLists;
                } else if (first != sentinel) {
                    ++(*count);
                }
            }
            std::byte* parent = nullptr;
            if (!Read(capture.bones[index] + kBoneParentOffset, parent) ||
                parent == nullptr) {
                continue;
            }
            ++parentLinks;
            for (std::size_t candidate = 0; candidate < capture.count;
                 ++candidate) {
                if (capture.bones[candidate] != parent) {
                    continue;
                }
                ++knownParents;
                if (candidate < index) {
                    ++earlierParents;
                }
                break;
            }
        }
        log::Write(log::Level::info,
                   std::wstring(label) + L" bone probe: bones=" +
                       std::to_wstring(capture.count) + L", affine=" +
                       std::to_wstring(affineCount) + L", uniform-rigid=" +
                       std::to_wstring(uniformRigidCount) +
                       L", parent-links/known/earlier=" +
                       std::to_wstring(parentLinks) + L"/" +
                       std::to_wstring(knownParents) + L"/" +
                       std::to_wstring(earlierParents) +
                       L", bones-with-attached-nodes/children=" +
                       std::to_wstring(bonesWithAttachedNodes) + L"/" +
                       std::to_wstring(bonesWithChildren) +
                       L", invalid-lists=" +
                       std::to_wstring(invalidLists) + L".");
        probe.topologyLogged = true;
    }
    return true;
}

void ResetBodyPose() noexcept {
    g_bodyPose.mesh = nullptr;
    g_bodyPose.count = 0;
    g_bodyPose.history.Reset();
}

bool HasSafeBoneOrder(const BoneCapture& capture) noexcept {
    for (std::size_t index = 0; index < capture.count; ++index) {
        std::byte* parent = nullptr;
        if (!Read(capture.bones[index] + kBoneParentOffset, parent)) {
            return false;
        }
        if (parent == nullptr) {
            continue;
        }
        for (std::size_t candidate = 0; candidate < capture.count;
             ++candidate) {
            if (capture.bones[candidate] == parent && candidate >= index) {
                return false;
            }
        }
    }
    return true;
}

bool SamePoseIdentity(const BoneCapture& capture) noexcept {
    if (g_bodyPose.mesh != capture.mesh ||
        g_bodyPose.count != capture.count) {
        return false;
    }
    for (std::size_t index = 0; index < capture.count; ++index) {
        if (g_bodyPose.bones[index] != capture.bones[index]) {
            return false;
        }
    }
    return true;
}

// Diagnostics only: how far PlayerBody bone locals move in one tick, measured
// against the limits that used to reset the pose history.
void ObserveBodyBoneMotion(const BoneCapture& capture) noexcept {
    if (g_previousBodyMesh == capture.mesh &&
        g_previousBodyBoneCount == capture.count) {
        float maximumPosition = 0.0F;
        float maximumOrientation = 0.0F;
        for (std::size_t index = 0; index < capture.count; ++index) {
            const root::Matrix& older = g_previousBodyBones[index];
            const root::Matrix& newer = capture.matrices[index];
            if (SameMatrixBits(older, newer)) {
                continue;
            }
            const float dx = newer.values[3] - older.values[3];
            const float dy = newer.values[7] - older.values[7];
            const float dz = newer.values[11] - older.values[11];
            maximumPosition = std::max(
                maximumPosition, std::sqrt(dx * dx + dy * dy + dz * dz));
            maximumOrientation = std::max(
                maximumOrientation, root::OrientationDistance(older, newer));
        }
        g_bodyMaximumPositionDelta =
            std::max(g_bodyMaximumPositionDelta, maximumPosition);
        g_bodyMaximumOrientationDelta =
            std::max(g_bodyMaximumOrientationDelta, maximumOrientation);
        if (maximumPosition > root::History::kMaximumPositionDeltaPerTick) {
            ++g_bodyTicksOverPositionLimit;
        }
        if (maximumOrientation >
            root::History::kMaximumOrientationDeltaPerTick) {
            ++g_bodyTicksOverOrientationLimit;
        }
    }
    g_previousBodyMesh = capture.mesh;
    g_previousBodyBoneCount = capture.count;
    std::memcpy(g_previousBodyBones.data(), capture.matrices.data(),
                capture.count * sizeof(root::Matrix));
}

void PublishBodyPose(const BoneCapture& capture,
                     const std::uint64_t tickSerial,
                     const std::uint64_t clockGeneration) noexcept {
    if (!g_poseInterpolationEnabled) {
        return;
    }
    if (g_diagnosticsEnabled) {
        ObserveBodyBoneMotion(capture);
    }
    if (!SamePoseIdentity(capture)) {
        ResetBodyPose();
        if (!HasSafeBoneOrder(capture)) {
            ++g_posePublishResets;
            return;
        }
        g_bodyPose.mesh = capture.mesh;
        g_bodyPose.count = capture.count;
        for (std::size_t index = 0; index < capture.count; ++index) {
            g_bodyPose.bones[index] = capture.bones[index];
        }
        // Parents outside the vector (the mesh node) count as roots.
        g_bodyParentsValid = true;
        for (std::size_t index = 0; index < capture.count; ++index) {
            std::byte* parent = nullptr;
            g_bodyParents[index] = -1;
            if (!Read(capture.bones[index] + kBoneParentOffset, parent)) {
                g_bodyParentsValid = false;
                break;
            }
            for (std::size_t candidate = 0;
                 parent != nullptr && candidate < index; ++candidate) {
                if (capture.bones[candidate] == parent) {
                    g_bodyParents[index] =
                        static_cast<std::int16_t>(candidate);
                    break;
                }
            }
        }
        g_attachment = {};
        g_attachmentSerial = 0;
    }
    // Bone locals are relative to the entity root, whose own history snaps on
    // teleports. A large local change between two consecutive ticks is fast
    // animation (arm swings, item use), so it is interpolated like any other
    // adjacent pair, as the skeletal-prop path already does.
    const pose::PublishKind published = g_bodyPose.history.Publish(
        tickSerial, clockGeneration, capture.matrices.data(), capture.count,
        pose::DiscontinuityMode::allowConsecutive);
    if (published == pose::PublishKind::invalid) {
        ++g_posePublishResets;
        ResetBodyPose();
        return;
    }
    ++g_posePublished;
    if (published != pose::PublishKind::appended) {
        ++g_posePublishResets;
    }
}

bool ValidateBodyPoseIdentity() noexcept {
    if (g_body.entity == nullptr || g_bodyPose.mesh == nullptr ||
        g_bodyPose.count == 0U) {
        return false;
    }
    std::string name;
    if (ReadKind(g_body.entity, name) != Kind::body) {
        return false;
    }
    std::byte* mesh = nullptr;
    std::byte** begin = nullptr;
    std::byte** end = nullptr;
    if (!Read(g_body.entity + kEntityFallbackNodeOffset, mesh) ||
        mesh != g_bodyPose.mesh ||
        !Read(mesh + kMeshBonesBeginOffset, begin) ||
        !Read(mesh + kMeshBonesEndOffset, end) || begin == nullptr ||
        end < begin) {
        return false;
    }
    const auto byteCount = reinterpret_cast<std::uintptr_t>(end) -
                           reinterpret_cast<std::uintptr_t>(begin);
    if (byteCount != g_bodyPose.count * sizeof(std::byte*) ||
        !IsAccessible(begin, byteCount, false)) {
        return false;
    }
    return std::memcmp(begin, g_bodyPose.bones.data(), byteCount) == 0;
}

bool RestoreBodyPose() noexcept {
    if (!g_appliedBodyPose.active) {
        return true;
    }
    bool restored = true;
    for (std::size_t index = 0; index < g_appliedBodyPose.count; ++index) {
        std::byte* const bone = g_appliedBodyPose.bones[index];
        if (bone == nullptr) {
            continue;
        }
        if (!IsAccessible(bone + kBoneLocalMatrixOffset,
                          sizeof(root::Matrix), true)) {
            restored = false;
            continue;
        }
        g_boneSetMatrix(bone, &g_appliedBodyPose.authoritative[index], false);
    }
    for (std::size_t index = 0; index < g_appliedBodyPose.count; ++index) {
        if (g_appliedBodyPose.bones[index] == nullptr) {
            continue;
        }
        root::Matrix actual{};
        if (!Read(g_appliedBodyPose.bones[index] + kBoneLocalMatrixOffset,
                  actual) ||
            !root::NearlyEqual(actual,
                               g_appliedBodyPose.authoritative[index],
                               2.0e-4F)) {
            restored = false;
        }
    }
    if (g_bodyPose.mesh != nullptr &&
        IsAccessible(g_bodyPose.mesh, sizeof(void*), false)) {
        g_nodeMatrixChanged(g_bodyPose.mesh, true);
    }
    g_appliedBodyPose.active = false;
    return restored;
}

bool ApplyBodyPose(const clock::Sample& sample) noexcept {
    if (!g_poseInterpolationEnabled) {
        return false;
    }
    const pose::SampleResult interpolated =
        g_bodyPose.history.Evaluate(sample);
    if (interpolated.kind != pose::SampleKind::applied) {
        return false;
    }
    if (!ValidateBodyPoseIdentity() ||
        interpolated.pose.count != g_bodyPose.count) {
        ++g_poseApplyFailures;
        ResetBodyPose();
        return false;
    }

    g_appliedBodyPose.active = false;
    g_appliedBodyPose.count = g_bodyPose.count;
    bool anyMoved = false;
    for (std::size_t index = 0; index < g_bodyPose.count; ++index) {
        std::byte* const bone = g_bodyPose.bones[index];
        root::Matrix& authoritative = g_appliedBodyPose.authoritative[index];
        root::TransformInfo info{};
        if (!IsAccessible(bone + kBoneLocalMatrixOffset,
                          sizeof(root::Matrix), true)) {
            ++g_poseApplyFailures;
            ResetBodyPose();
            return false;
        }
        std::memcpy(&authoritative, bone + kBoneLocalMatrixOffset,
                    sizeof(root::Matrix));
        if (!root::Analyze(authoritative, info)) {
            ++g_poseApplyFailures;
            ResetBodyPose();
            return false;
        }
        g_appliedBodyPose.bones[index] = bone;
        anyMoved = anyMoved ||
            !SameMatrixBits(authoritative, interpolated.pose.matrices[index]);
    }
    // Every bone is written once any bone moved: an unwritten child keeps its
    // cached 60 Hz world transform even though its parent was interpolated
    // (fingers under a swinging forearm), so per-bone skipping is not safe.
    if (!anyMoved) {
        return false;
    }

    g_appliedBodyPose.active = true;
    for (std::size_t index = 0; index < g_bodyPose.count; ++index) {
        if (g_appliedBodyPose.bones[index] == nullptr) {
            continue;
        }
        g_boneSetMatrix(g_bodyPose.bones[index],
                        &interpolated.pose.matrices[index], false);
        root::Matrix actual{};
        if (!Read(g_bodyPose.bones[index] + kBoneLocalMatrixOffset, actual) ||
            !root::NearlyEqual(actual, interpolated.pose.matrices[index],
                               2.0e-4F)) {
            ++g_poseApplyFailures;
            static_cast<void>(RestoreBodyPose());
            ResetBodyPose();
            return false;
        }
    }
    // A moving entity root refreshes the mesh's cached skinning palette as a
    // side effect. With a resting root (scripted first-person animations)
    // nothing else does, so mirror the native mesh invalidation here.
    g_nodeMatrixChanged(g_bodyPose.mesh, true);
    ++g_poseApplied;
    return true;
}

void ResetScriptedPose(ScriptedPoseEntry& entry,
                       const bool clearEntity) noexcept {
    entry.pose.mesh = nullptr;
    entry.pose.count = 0;
    entry.pose.history.Reset();
    entry.applied = {};
    entry.probe = {};
    if (clearEntity) {
        entry.entity = nullptr;
    }
}

bool SameScriptedPoseIdentity(const ScriptedPoseEntry& entry,
                              const BoneCapture& capture) noexcept {
    if (entry.pose.mesh != capture.mesh ||
        entry.pose.count != capture.count) {
        return false;
    }
    for (std::size_t index = 0; index < capture.count; ++index) {
        if (entry.pose.bones[index] != capture.bones[index]) {
            return false;
        }
    }
    return true;
}

void PublishScriptedPose(ScriptedPoseEntry& entry,
                         const BoneCapture& capture,
                         const std::uint64_t tickSerial,
                         const std::uint64_t clockGeneration) noexcept {
    if (!g_poseInterpolationEnabled) {
        return;
    }
    if (!SameScriptedPoseIdentity(entry, capture)) {
        entry.pose.mesh = nullptr;
        entry.pose.count = 0;
        entry.pose.history.Reset();
        if (!HasSafeBoneOrder(capture)) {
            ++entry.publishResets;
            return;
        }
        entry.pose.mesh = capture.mesh;
        entry.pose.count = capture.count;
        for (std::size_t index = 0; index < capture.count; ++index) {
            entry.pose.bones[index] = capture.bones[index];
        }
    }
    const pose::PublishKind published = entry.pose.history.Publish(
        tickSerial, clockGeneration, capture.matrices.data(), capture.count);
    if (published == pose::PublishKind::invalid) {
        ++entry.publishResets;
        entry.pose.mesh = nullptr;
        entry.pose.count = 0;
        entry.pose.history.Reset();
        return;
    }
    ++entry.published;
    if (published != pose::PublishKind::appended) {
        ++entry.publishResets;
    }
}

bool ValidateScriptedPoseIdentity(ScriptedPoseEntry& entry) noexcept {
    if (entry.entity == nullptr || entry.pose.mesh == nullptr ||
        entry.pose.count == 0U) {
        return false;
    }
    std::string name;
    if (ReadKind(entry.entity, name) != Kind::scriptedPose ||
        entry.name == nullptr || name != entry.name) {
        return false;
    }
    std::byte* mesh = nullptr;
    std::byte** begin = nullptr;
    std::byte** end = nullptr;
    if (!Read(entry.entity + kEntityFallbackNodeOffset, mesh) ||
        mesh != entry.pose.mesh ||
        !Read(mesh + kMeshBonesBeginOffset, begin) ||
        !Read(mesh + kMeshBonesEndOffset, end) || begin == nullptr ||
        end < begin) {
        return false;
    }
    const auto byteCount = reinterpret_cast<std::uintptr_t>(end) -
                           reinterpret_cast<std::uintptr_t>(begin);
    return byteCount == entry.pose.count * sizeof(std::byte*) &&
           IsAccessible(begin, byteCount, false) &&
           std::memcmp(begin, entry.pose.bones.data(), byteCount) == 0;
}

bool RestoreScriptedPose(ScriptedPoseEntry& entry) noexcept {
    if (!entry.applied.active) {
        return true;
    }
    bool restored = true;
    for (std::size_t index = 0; index < entry.applied.count; ++index) {
        std::byte* const bone = entry.applied.bones[index];
        if (!IsAccessible(bone + kBoneLocalMatrixOffset,
                          sizeof(root::Matrix), true)) {
            restored = false;
            continue;
        }
        g_boneSetMatrix(bone, &entry.applied.authoritative[index], false);
    }
    for (std::size_t index = 0; index < entry.applied.count; ++index) {
        root::Matrix actual{};
        if (!Read(entry.applied.bones[index] + kBoneLocalMatrixOffset,
                  actual) ||
            !root::NearlyEqual(actual, entry.applied.authoritative[index],
                               2.0e-4F)) {
            restored = false;
        }
    }
    if (entry.pose.mesh != nullptr &&
        IsAccessible(entry.pose.mesh, sizeof(void*), false)) {
        // cMeshEntity::PostUpdateLogic performs this same invalidation after
        // authoritative animation updates. Without it, static scripted
        // entities can keep rendering a previously cached skinning palette.
        g_nodeMatrixChanged(entry.pose.mesh, true);
    } else {
        restored = false;
    }
    entry.applied.active = false;
    return restored;
}

bool ApplyScriptedPose(ScriptedPoseEntry& entry,
                       const clock::Sample& sample) noexcept {
    if (!g_poseInterpolationEnabled) {
        return false;
    }
    const pose::SampleResult interpolated = entry.pose.history.Evaluate(sample);
    if (interpolated.kind != pose::SampleKind::applied) {
        return false;
    }
    if (!ValidateScriptedPoseIdentity(entry) ||
        interpolated.pose.count != entry.pose.count) {
        ++entry.applyFailures;
        ResetScriptedPose(entry, true);
        return false;
    }

    entry.applied = {};
    entry.applied.count = entry.pose.count;
    for (std::size_t index = 0; index < entry.pose.count; ++index) {
        std::byte* const bone = entry.pose.bones[index];
        root::TransformInfo info{};
        if (!IsAccessible(bone + kBoneLocalMatrixOffset,
                          sizeof(root::Matrix), true) ||
            !Read(bone + kBoneLocalMatrixOffset,
                  entry.applied.authoritative[index]) ||
            !root::Analyze(entry.applied.authoritative[index], info)) {
            ++entry.applyFailures;
            ResetScriptedPose(entry, true);
            return false;
        }
        entry.applied.bones[index] = bone;
    }

    entry.applied.active = true;
    for (std::size_t index = 0; index < entry.pose.count; ++index) {
        g_boneSetMatrix(entry.pose.bones[index],
                        &interpolated.pose.matrices[index], false);
        root::Matrix actual{};
        if (!Read(entry.pose.bones[index] + kBoneLocalMatrixOffset, actual) ||
            !root::NearlyEqual(actual, interpolated.pose.matrices[index],
                               2.0e-4F)) {
            ++entry.applyFailures;
            static_cast<void>(RestoreScriptedPose(entry));
            ResetScriptedPose(entry, true);
            return false;
        }
    }
    // Bone setters refresh bone world matrices parent-first, but NPC/prop
    // meshes cache their render skinning data. Mirror the mesh invalidation
    // issued by cMeshEntity::PostUpdateLogic so the interpolated pose reaches
    // that render cache before the listener viewport is drawn.
    g_nodeMatrixChanged(entry.pose.mesh, true);
    ++entry.appliedCount;
    if (!entry.activationLogged) {
        log::Write(log::Level::info,
                   std::wstring(entry.label) +
                       L" scripted skeletal-pose interpolation active on "
                       L"the camera presentation clock.");
        entry.activationLogged = true;
    }
    return true;
}

bool ReadArticulatedBodyCount(std::byte* const entity,
                              std::size_t& count) noexcept {
    count = 0;
    std::byte** begin = nullptr;
    std::byte** end = nullptr;
    if (entity == nullptr ||
        !Read(entity + kEntityNodesBeginOffset, begin) ||
        !Read(entity + kEntityNodesEndOffset, end)) {
        return false;
    }
    if (begin == end) {
        count = 0U;
        return true;
    }
    if (begin == nullptr || end < begin) {
        return false;
    }
    const auto bytes = reinterpret_cast<std::uintptr_t>(end) -
                       reinterpret_cast<std::uintptr_t>(begin);
    if (bytes % sizeof(std::byte*) != 0U ||
        !IsAccessible(begin, bytes, false)) {
        return false;
    }
    count = bytes / sizeof(std::byte*);
    return count >= kMinimumArticulatedPropBodies &&
           count <= kMaximumArticulatedPropBodies;
}

// True when the object's vtable is cLuxAgent's own: its OnUpdate slot is the
// function we detour and its mesh getter is the proven +0x940 accessor.
bool ValidAgentVtable(std::byte* const entity) noexcept {
    std::byte* vtable = nullptr;
    void* getMesh = nullptr;
    void* onUpdate = nullptr;
    return Read(entity, vtable) && vtable != nullptr &&
           IsAccessible(vtable, kVtableOnUpdateSlot + sizeof(void*), false) &&
           Read(vtable + kVtableGetMeshSlot, getMesh) &&
           Read(vtable + kVtableOnUpdateSlot, onUpdate) &&
           getMesh == g_executableBase + kLuxAgentGetMeshRva &&
           onUpdate == g_executableBase + kLuxAgentOnUpdateRva;
}

bool HasArticulatedBoneTopology(std::byte* const entity,
                                const std::size_t meshOffset) noexcept {
    std::byte* mesh = nullptr;
    std::byte** begin = nullptr;
    std::byte** end = nullptr;
    if (entity == nullptr ||
        !Read(entity + meshOffset, mesh) || mesh == nullptr ||
        !Read(mesh + kMeshBonesBeginOffset, begin) ||
        !Read(mesh + kMeshBonesEndOffset, end) || begin == nullptr ||
        end < begin) {
        return false;
    }
    const auto bytes = reinterpret_cast<std::uintptr_t>(end) -
                       reinterpret_cast<std::uintptr_t>(begin);
    const std::size_t count = bytes / sizeof(std::byte*);
    return bytes % sizeof(std::byte*) == 0U && count != 0U &&
           count <= kMaximumBones && IsAccessible(begin, bytes, false);
}

void ClearArticulatedPose(ArticulatedPoseEntry& entry) noexcept {
    entry.entity = nullptr;
    entry.name = {};
    entry.lastSeenSerial = 0;
    entry.bodyCount = 0;
    entry.meshOffset = kEntityFallbackNodeOffset;
    entry.agent = false;
    entry.meshRoot.Reset();
    entry.meshRootApplied = false;
    entry.staged = false;
    entry.interpolateCurrentInterval = false;
    entry.activationLogged = false;
    entry.probe = {};
    entry.pose = {};
    entry.applied = {};
}

void StageArticulatedProp(std::byte* const entity,
                          const bool agent = false) noexcept {
    if (!g_articulatedPoseInterpolationEnabled || entity == nullptr) {
        return;
    }
    std::size_t bodyCount = 0;
    std::string name;
    const std::size_t meshOffset =
        agent ? kAgentMeshOffset : kEntityFallbackNodeOffset;
    if (agent) {
        if (!ValidAgentVtable(entity)) {
            return;
        }
    } else if (!ValidEntityVtable(entity) ||
               !ReadArticulatedBodyCount(entity, bodyCount)) {
        return;
    }
    if (!HasArticulatedBoneTopology(entity, meshOffset) ||
        !ReadStringObject(entity + kEntityNameOffset, name)) {
        return;
    }
    if (!agent) {
        std::string classifiedName;
        const Kind kind = ReadKind(entity, classifiedName);
        if (kind == Kind::hands || kind == Kind::body) {
            return;
        }
    }

    ArticulatedPoseEntry* available = nullptr;
    for (ArticulatedPoseEntry& candidate : g_articulatedPoses) {
        if (candidate.entity == entity) {
            if (name != candidate.name.data()) {
                ClearArticulatedPose(candidate);
            }
            available = &candidate;
            break;
        }
        if (available == nullptr && candidate.entity == nullptr) {
            available = &candidate;
        }
    }
    if (available == nullptr) {
        ++g_articulatedRegistryOverflows;
        return;
    }
    if (available->entity == nullptr) {
        available->entity = entity;
        available->agent = agent;
        available->meshOffset = meshOffset;
        const std::size_t copied =
            std::min(name.size(), available->name.size() - 1U);
        std::memcpy(available->name.data(), name.data(), copied);
        available->name[copied] = '\0';
    }
    available->bodyCount = bodyCount;
    available->staged = true;
}

bool SameArticulatedPoseIdentity(const ArticulatedPoseEntry& entry,
                                 const BoneCapture& capture) noexcept {
    if (entry.pose.mesh != capture.mesh ||
        entry.pose.count != capture.count) {
        return false;
    }
    for (std::size_t index = 0; index < capture.count; ++index) {
        if (entry.pose.bones[index] != capture.bones[index]) {
            return false;
        }
    }
    return true;
}

void PublishArticulatedPose(ArticulatedPoseEntry& entry,
                            const BoneCapture& capture,
                            const std::uint64_t tickSerial,
                            const std::uint64_t clockGeneration) noexcept {
    const bool sameIdentity = SameArticulatedPoseIdentity(entry, capture);
    if (!sameIdentity) {
        entry.pose = {};
        if (!HasSafeBoneOrder(capture)) {
            ++entry.publishResets;
            return;
        }
        entry.pose.mesh = capture.mesh;
        entry.pose.count = capture.count;
        for (std::size_t index = 0; index < capture.count; ++index) {
            entry.pose.bones[index] = capture.bones[index];
        }
    }
    const pose::PublishKind published = entry.pose.history.Publish(
        tickSerial, clockGeneration, capture.matrices.data(), capture.count,
        pose::DiscontinuityMode::allowConsecutive);
    if (published == pose::PublishKind::invalid) {
        ++entry.publishResets;
        entry.pose = {};
        entry.interpolateCurrentInterval = false;
        return;
    }
    entry.interpolateCurrentInterval = sameIdentity &&
        published == pose::PublishKind::appended &&
        capture.changedFromPrevious;
    ++entry.published;
    if (published != pose::PublishKind::appended) {
        ++entry.publishResets;
    }
    if (published == pose::PublishKind::resetDiscontinuity) {
        ++entry.discontinuityResets;
    }
}

bool ValidateArticulatedPoseIdentity(
    const ArticulatedPoseEntry& entry) noexcept {
    if (entry.entity == nullptr || entry.pose.mesh == nullptr ||
        entry.pose.count == 0U ||
        !(entry.agent ? ValidAgentVtable(entry.entity)
                      : ValidEntityVtable(entry.entity))) {
        return false;
    }
    std::string name;
    std::size_t bodyCount = 0;
    if (!ReadStringObject(entry.entity + kEntityNameOffset, name) ||
        name != entry.name.data() ||
        (!entry.agent &&
         !ReadArticulatedBodyCount(entry.entity, bodyCount))) {
        return false;
    }
    std::byte* mesh = nullptr;
    std::byte** begin = nullptr;
    std::byte** end = nullptr;
    if (!Read(entry.entity + entry.meshOffset, mesh) ||
        mesh != entry.pose.mesh ||
        !Read(mesh + kMeshBonesBeginOffset, begin) ||
        !Read(mesh + kMeshBonesEndOffset, end) || begin == nullptr ||
        end < begin) {
        return false;
    }
    const auto bytes = reinterpret_cast<std::uintptr_t>(end) -
                       reinterpret_cast<std::uintptr_t>(begin);
    return bytes == entry.pose.count * sizeof(std::byte*) &&
           IsAccessible(begin, bytes, false) &&
           std::memcmp(begin, entry.pose.bones.data(), bytes) == 0;
}

bool RestoreArticulatedPose(ArticulatedPoseEntry& entry) noexcept {
    if (!entry.applied.active) {
        return true;
    }
    bool restored = true;
    for (std::size_t index = 0; index < entry.applied.count; ++index) {
        std::byte* const bone = entry.applied.bones[index];
        if (bone == nullptr) {
            continue;
        }
        if (!IsAccessible(bone + kBoneLocalMatrixOffset,
                          sizeof(root::Matrix), true)) {
            restored = false;
            continue;
        }
        g_boneSetMatrix(bone, &entry.applied.authoritative[index], false);
    }
    for (std::size_t index = 0; index < entry.applied.count; ++index) {
        if (entry.applied.bones[index] == nullptr) {
            continue;
        }
        root::Matrix actual{};
        if (!Read(entry.applied.bones[index] + kBoneLocalMatrixOffset,
                  actual) ||
            !root::NearlyEqual(actual, entry.applied.authoritative[index],
                               2.0e-4F)) {
            restored = false;
        }
    }
    if (entry.meshRootApplied) {
        entry.meshRootApplied = false;
        if (entry.pose.mesh != nullptr &&
            IsAccessible(entry.pose.mesh + kNodeLocalMatrixOffset,
                         sizeof(root::Matrix), true)) {
            std::memcpy(entry.pose.mesh + kNodeLocalMatrixOffset,
                        &entry.meshRootAuthoritative, sizeof(root::Matrix));
        } else {
            restored = false;
        }
    }
    if (entry.pose.mesh != nullptr &&
        IsAccessible(entry.pose.mesh, sizeof(void*), false)) {
        g_nodeMatrixChanged(entry.pose.mesh, true);
    } else {
        restored = false;
    }
    entry.applied.active = false;
    return restored;
}

bool ApplyArticulatedPose(ArticulatedPoseEntry& entry,
                          const clock::Sample& sample) noexcept {
    if (!entry.interpolateCurrentInterval) {
        ++entry.unchangedRenderSkips;
        return false;
    }
    const pose::SampleResult interpolated = entry.pose.history.Evaluate(sample);
    if (interpolated.kind != pose::SampleKind::applied) {
        return false;
    }
    if (!ValidateArticulatedPoseIdentity(entry) ||
        interpolated.pose.count != entry.pose.count) {
        ++entry.applyFailures;
        ClearArticulatedPose(entry);
        return false;
    }

    entry.applied.active = false;
    entry.applied.count = entry.pose.count;
    std::size_t movedBones = 0;
    for (std::size_t index = 0; index < entry.pose.count; ++index) {
        std::byte* const bone = entry.pose.bones[index];
        root::Matrix& authoritative = entry.applied.authoritative[index];
        root::TransformInfo info{};
        if (!IsAccessible(bone + kBoneLocalMatrixOffset,
                          sizeof(root::Matrix), true)) {
            ++entry.applyFailures;
            ClearArticulatedPose(entry);
            return false;
        }
        std::memcpy(&authoritative, bone + kBoneLocalMatrixOffset,
                    sizeof(root::Matrix));
        if (!root::Analyze(authoritative, info)) {
            ++entry.applyFailures;
            ClearArticulatedPose(entry);
            return false;
        }
        entry.applied.bones[index] = bone;
        movedBones +=
            SameMatrixBits(authoritative, interpolated.pose.matrices[index])
                ? 0U
                : 1U;
    }
    // Every bone is written once any bone moved: an unwritten child keeps its
    // cached 60 Hz world transform even though its parent was interpolated,
    // so per-bone skipping is not safe.
    if (movedBones == 0U) {
        ++entry.unchangedRenderSkips;
        return false;
    }
    movedBones = entry.pose.count;

    entry.applied.active = true;
    for (std::size_t index = 0; index < entry.pose.count; ++index) {
        if (entry.applied.bones[index] == nullptr) {
            continue;
        }
        g_boneSetMatrix(entry.pose.bones[index],
                        &interpolated.pose.matrices[index], false);
        root::Matrix actual{};
        if (!Read(entry.pose.bones[index] + kBoneLocalMatrixOffset, actual) ||
            !root::NearlyEqual(actual, interpolated.pose.matrices[index],
                               2.0e-4F)) {
            ++entry.applyFailures;
            static_cast<void>(RestoreArticulatedPose(entry));
            ClearArticulatedPose(entry);
            return false;
        }
    }
    if (entry.agent) {
        // Move the agent's mesh node to its interpolated tick transform as
        // well; the invalidation below then covers both node and bones.
        const root::SampleResult rootSample =
            entry.meshRoot.Evaluate(sample);
        std::byte* const meshMatrix =
            entry.pose.mesh + kNodeLocalMatrixOffset;
        if (rootSample.kind == root::SampleKind::applied &&
            IsAccessible(meshMatrix, sizeof(root::Matrix), true)) {
            std::memcpy(&entry.meshRootAuthoritative, meshMatrix,
                        sizeof(root::Matrix));
            std::memcpy(meshMatrix, &rootSample.matrix,
                        sizeof(root::Matrix));
            entry.meshRootApplied = true;
            ++g_agentRootApplied;
        }
    }
    g_nodeMatrixChanged(entry.pose.mesh, true);
    ++entry.appliedCount;
    g_articulatedBonesWritten += movedBones;
    if (!entry.activationLogged) {
        std::wstring wideName;
        for (const char character : entry.name) {
            if (character == '\0') break;
            wideName.push_back(static_cast<unsigned char>(character));
        }
        log::Write(log::Level::info,
                   L"Skeletal prop pose interpolation active: " +
                       wideName + L" (bodies=" +
                       std::to_wstring(entry.bodyCount) + L", bones=" +
                       std::to_wstring(entry.pose.count) + L").");
        entry.activationLogged = true;
    }
    return true;
}

void MaybeReport() {
    if (!g_diagnosticsEnabled) {
        return;
    }
    const ULONGLONG now = GetTickCount64();
    if (g_lastReportMilliseconds == 0U) {
        g_lastReportMilliseconds = now;
        return;
    }
    if (now - g_lastReportMilliseconds < 5000U) {
        return;
    }

    struct WorldBodyReport {
        std::array<char, kMaximumNameLength + 1U> name{};
        std::uint64_t callbacks = 0;
        std::uint64_t changedCallbacks = 0;
    };
    std::array<WorldBodyReport, kMaximumWorldBodyProbes> worldBodies{};
    std::size_t worldBodyCount = 0;
    std::uint64_t worldCallbacks = 0;
    std::uint64_t worldChangedCallbacks = 0;
    while (g_worldBodyProbeLock.test_and_set(std::memory_order_acquire)) {
        SwitchToThread();
    }
    for (WorldBodyProbe& probe : g_worldBodyProbes) {
        if (probe.body != nullptr && probe.callbacks != 0U) {
            WorldBodyReport& report = worldBodies[worldBodyCount++];
            report.name = probe.name;
            report.callbacks = probe.callbacks;
            report.changedCallbacks = probe.changedCallbacks;
            worldCallbacks += probe.callbacks;
            worldChangedCallbacks += probe.changedCallbacks;
        }
        probe = {};
    }
    g_worldBodyProbeLock.clear(std::memory_order_release);
    std::sort(worldBodies.begin(), worldBodies.begin() + worldBodyCount,
              [](const WorldBodyReport& left, const WorldBodyReport& right) {
                  if (left.changedCallbacks != right.changedCallbacks) {
                      return left.changedCallbacks > right.changedCallbacks;
                  }
                  return left.callbacks > right.callbacks;
              });
    std::wstring worldBodySummary;
    constexpr std::size_t maximumReportedWorldBodies = 12U;
    const std::size_t reportedWorldBodies =
        std::min(worldBodyCount, maximumReportedWorldBodies);
    for (std::size_t index = 0; index < reportedWorldBodies; ++index) {
        if (!worldBodySummary.empty()) {
            worldBodySummary += L", ";
        }
        for (const char character : worldBodies[index].name) {
            if (character == '\0') {
                break;
            }
            worldBodySummary.push_back(static_cast<unsigned char>(character));
        }
        worldBodySummary += L"(" +
            std::to_wstring(worldBodies[index].callbacks) + L"/" +
            std::to_wstring(worldBodies[index].changedCallbacks) + L")";
    }
    log::Write(log::Level::info,
               L"Newton body transform probe: bodies/callbacks/changed/"
               L"invalid/lock-drop/registry-overflow=" +
                   std::to_wstring(worldBodyCount) + L"/" +
                   std::to_wstring(worldCallbacks) + L"/" +
                   std::to_wstring(worldChangedCallbacks) + L"/" +
                   std::to_wstring(g_worldBodyProbeInvalid.exchange(0U)) +
                   L"/" +
                   std::to_wstring(g_worldBodyProbeLockDrops.exchange(0U)) +
                   L"/" +
                   std::to_wstring(g_worldBodyProbeOverflows.exchange(0U)) +
                   L"; top name(callbacks/changed): " +
                   (worldBodySummary.empty() ? L"<none>" : worldBodySummary) +
                   L".");

    struct EntityRootReport {
        std::array<char, kMaximumNameLength + 1U> name{};
        std::uintptr_t callerRva = 0;
        std::uintptr_t vtableRva = 0;
        std::uint64_t callbacks = 0;
        std::uint64_t changedCallbacks = 0;
    };
    std::array<EntityRootReport, kMaximumEntityRootProbes> entityRoots{};
    std::size_t entityRootCount = 0;
    std::uint64_t entityRootCallbacks = 0;
    std::uint64_t entityRootChangedCallbacks = 0;
    while (g_entityRootProbeLock.test_and_set(std::memory_order_acquire)) {
        SwitchToThread();
    }
    for (EntityRootProbe& probe : g_entityRootProbes) {
        if (probe.entity != nullptr && probe.callbacks != 0U) {
            EntityRootReport& report = entityRoots[entityRootCount++];
            report.name = probe.name;
            report.callerRva = probe.callerRva;
            report.vtableRva = probe.vtableRva;
            report.callbacks = probe.callbacks;
            report.changedCallbacks = probe.changedCallbacks;
            entityRootCallbacks += probe.callbacks;
            entityRootChangedCallbacks += probe.changedCallbacks;
        }
        probe = {};
    }
    g_entityRootProbeLock.clear(std::memory_order_release);
    std::sort(entityRoots.begin(), entityRoots.begin() + entityRootCount,
              [](const EntityRootReport& left,
                 const EntityRootReport& right) {
                  if (left.changedCallbacks != right.changedCallbacks) {
                      return left.changedCallbacks > right.changedCallbacks;
                  }
                  return left.callbacks > right.callbacks;
              });
    std::wstring entityRootSummary;
    constexpr std::size_t maximumReportedEntityRoots = 12U;
    const std::size_t reportedEntityRoots =
        std::min(entityRootCount, maximumReportedEntityRoots);
    for (std::size_t index = 0; index < reportedEntityRoots; ++index) {
        if (!entityRootSummary.empty()) {
            entityRootSummary += L", ";
        }
        for (const char character : entityRoots[index].name) {
            if (character == '\0') {
                break;
            }
            entityRootSummary.push_back(
                static_cast<unsigned char>(character));
        }
        entityRootSummary += L"(" +
            std::to_wstring(entityRoots[index].callbacks) + L"/" +
            std::to_wstring(entityRoots[index].changedCallbacks) +
            L", caller=" + Hex(entityRoots[index].callerRva) +
            L", vtable=" + Hex(entityRoots[index].vtableRva) + L")";
    }
    log::Write(log::Level::info,
               L"Generic entity SetMatrix probe: entities/callbacks/changed/"
               L"invalid/lock-drop/registry-overflow=" +
                   std::to_wstring(entityRootCount) + L"/" +
                   std::to_wstring(entityRootCallbacks) + L"/" +
                   std::to_wstring(entityRootChangedCallbacks) + L"/" +
                   std::to_wstring(g_entityRootProbeInvalid.exchange(0U)) +
                   L"/" +
                   std::to_wstring(g_entityRootProbeLockDrops.exchange(0U)) +
                   L"/" +
                   std::to_wstring(g_entityRootProbeOverflows.exchange(0U)) +
                   L"; top name(callbacks/changed): " +
                   (entityRootSummary.empty() ? L"<none>"
                                              : entityRootSummary) +
                   L".");
    if (g_worldInterpolationEnabled) {
        std::size_t activeWorldBodies = 0;
        for (const TrackedWorldBody& tracked : g_worldBodies) {
            activeWorldBodies += tracked.body != nullptr ? 1U : 0U;
        }
        log::Write(log::Level::info,
                   L"Newton rigid-body interpolation: active/published/reset, "
                   L"render applied/fail/restore-fail/stage-overflow=" +
                       std::to_wstring(activeWorldBodies) + L"/" +
                       std::to_wstring(g_worldPublished) + L"/" +
                       std::to_wstring(g_worldPublishResets) + L", " +
                       std::to_wstring(g_worldApplied) + L"/" +
                       std::to_wstring(g_worldApplyFailures) + L"/" +
                       std::to_wstring(g_worldRestoreFailures) + L"/" +
                       std::to_wstring(g_worldStageOverflows.exchange(0U)) +
                       L".");
    }
    log::Write(log::Level::info,
               L"Player-root cadence: hands/body/held-set-matrix=" +
                   std::to_wstring(g_handsCalls) + L"/" +
                   std::to_wstring(g_bodyCalls) + L"/" +
                   std::to_wstring(g_heldAttachmentCalls) +
                   L", invalid-matrix=" +
                   std::to_wstring(g_invalidMatrices) +
                   L", render applied/fail/restore-fail=" +
                   std::to_wstring(g_rootApplied) + L"/" +
                   std::to_wstring(g_rootApplyFailures) + L"/" +
                   std::to_wstring(g_rootRestoreFailures) + L".");
    log::Write(log::Level::info,
               L"Player-root failure detail: read-root/collect/inverse/"
               L"multiply/verify=" + std::to_wstring(g_readRootFailures) +
                   L"/" + std::to_wstring(g_collectNodeFailures) + L"/" +
                   std::to_wstring(g_inverseFailures) + L"/" +
                   std::to_wstring(g_multiplyFailures) + L"/" +
                   std::to_wstring(g_verifyFailures) + L".");
    log::Write(log::Level::info,
               L"Body-bone read-only cadence: captures/changed/invalid/count=" +
                   std::to_wstring(g_bodyBoneProbe.captures) + L"/" +
                   std::to_wstring(g_bodyBoneProbe.changedCaptures) + L"/" +
                   std::to_wstring(g_bodyBoneProbe.invalidCaptures) + L"/" +
                   std::to_wstring(g_bodyBoneProbe.boneCount) + L".");
    log::Write(log::Level::info,
               L"Hands-bone read-only cadence: captures/changed/invalid/count=" +
                   std::to_wstring(g_handsBoneProbe.captures) + L"/" +
                   std::to_wstring(g_handsBoneProbe.changedCaptures) + L"/" +
                   std::to_wstring(g_handsBoneProbe.invalidCaptures) + L"/" +
                   std::to_wstring(g_handsBoneProbe.boneCount) + L".");
    log::Write(log::Level::info,
               L"Held/accessory interpolation: render applied/fail/"
               L"restore-fail/registry-overflow=" +
                   std::to_wstring(g_heldApplied) + L"/" +
                   std::to_wstring(g_heldApplyFailures) + L"/" +
                   std::to_wstring(g_heldRestoreFailures) + L"/" +
                   std::to_wstring(g_heldRegistryOverflows) + L".");
    log::Write(log::Level::info,
               L"Scripted-root interpolation: set-matrix, render "
               L"applied/fail/restore-fail/registry-overflow=" +
                   std::to_wstring(g_scriptedRootCalls) + L", " +
                   std::to_wstring(g_scriptedRootApplied) + L"/" +
                   std::to_wstring(g_scriptedRootApplyFailures) + L"/" +
                   std::to_wstring(g_scriptedRootRestoreFailures) + L"/" +
                   std::to_wstring(g_scriptedRootRegistryOverflows) + L".");
    log::Write(log::Level::info,
               L"Inspection-root interpolation: set-matrix, render "
               L"applied/fail/restore-fail/registry-overflow=" +
                   std::to_wstring(g_inspectionRootCalls) + L", " +
                   std::to_wstring(g_inspectionApplied) + L"/" +
                   std::to_wstring(g_inspectionApplyFailures) + L"/" +
                   std::to_wstring(g_inspectionRestoreFailures) + L"/" +
                   std::to_wstring(g_inspectionRegistryOverflows) + L".");
    if (g_articulatedPoseInterpolationEnabled) {
        std::size_t active = 0;
        std::uint64_t captures = 0;
        std::uint64_t changed = 0;
        std::uint64_t invalid = 0;
        std::uint64_t published = 0;
        std::uint64_t resets = 0;
        std::uint64_t discontinuityResets = 0;
        std::uint64_t applied = 0;
        std::uint64_t applyFailures = 0;
        std::uint64_t restoreFailures = 0;
        std::uint64_t unchangedRenderSkips = 0;
        std::wstring summary;
        constexpr std::size_t maximumReported = 8U;
        for (const ArticulatedPoseEntry& entry : g_articulatedPoses) {
            if (entry.entity == nullptr) continue;
            ++active;
            captures += entry.probe.captures;
            changed += entry.probe.changedCaptures;
            invalid += entry.probe.invalidCaptures;
            published += entry.published;
            resets += entry.publishResets;
            discontinuityResets += entry.discontinuityResets;
            applied += entry.appliedCount;
            applyFailures += entry.applyFailures;
            restoreFailures += entry.restoreFailures;
            unchangedRenderSkips += entry.unchangedRenderSkips;
            if (active <= maximumReported) {
                if (!summary.empty()) summary += L", ";
                for (const char character : entry.name) {
                    if (character == '\0') break;
                    summary.push_back(static_cast<unsigned char>(character));
                }
                summary += L"(" + std::to_wstring(entry.bodyCount) + L"b/" +
                           std::to_wstring(entry.probe.boneCount) + L"j/" +
                           std::to_wstring(entry.probe.changedCaptures) +
                           L"c)";
            }
        }
        log::Write(log::Level::info,
                   L"Skeletal prop pose interpolation: active, captures/"
                   L"changed/invalid, published/reset/discontinuity-reset, "
                   L"render applied/unchanged-skip/fail/"
                   L"restore-fail/registry-overflow=" +
                       std::to_wstring(active) + L", " +
                       std::to_wstring(captures) + L"/" +
                       std::to_wstring(changed) + L"/" +
                       std::to_wstring(invalid) + L", " +
                       std::to_wstring(published) + L"/" +
                       std::to_wstring(resets) + L"/" +
                       std::to_wstring(discontinuityResets) + L", " +
                       std::to_wstring(applied) + L"/" +
                        std::to_wstring(unchangedRenderSkips) + L"/" +
                       std::to_wstring(applyFailures) + L"/" +
                       std::to_wstring(restoreFailures) + L"/" +
                       std::to_wstring(g_articulatedRegistryOverflows) +
                       L"; props(bodies/bones/changed): " +
                       (summary.empty() ? L"<none>" : summary) + L".");
    }
    if (g_poseInterpolationEnabled) {
        log::Write(log::Level::info,
                   L"Body-pose cadence: published/reset, render "
                   L"applied/fail/restore-fail=" +
                       std::to_wstring(g_posePublished) + L"/" +
                       std::to_wstring(g_posePublishResets) + L", " +
                       std::to_wstring(g_poseApplied) + L"/" +
                       std::to_wstring(g_poseApplyFailures) + L"/" +
                       std::to_wstring(g_poseRestoreFailures) +
                       L"; largest bone-local change per tick "
                       L"position/orientation=" +
                       std::to_wstring(g_bodyMaximumPositionDelta) + L"/" +
                       std::to_wstring(g_bodyMaximumOrientationDelta) +
                       L" rad, ticks over former limits=" +
                       std::to_wstring(g_bodyTicksOverPositionLimit) + L"/" +
                       std::to_wstring(g_bodyTicksOverOrientationLimit) +
                       L".");
        g_bodyMaximumPositionDelta = 0.0F;
        g_bodyMaximumOrientationDelta = 0.0F;
        g_bodyTicksOverPositionLimit = 0;
        g_bodyTicksOverOrientationLimit = 0;
        for (const ScriptedPoseEntry& entry : g_scriptedPoses) {
            log::Write(
                log::Level::info,
                std::wstring(entry.label) +
                    L" scripted-pose cadence: lookup/set-matrix/captures/changed/"
                    L"invalid/count=" +
                    std::to_wstring(entry.lookupCalls) + L"/" +
                    std::to_wstring(entry.setMatrixCalls) + L"/" +
                    std::to_wstring(entry.probe.captures) + L"/" +
                    std::to_wstring(entry.probe.changedCaptures) + L"/" +
                    std::to_wstring(entry.probe.invalidCaptures) + L"/" +
                    std::to_wstring(entry.probe.boneCount) +
                    L", published/reset=" +
                    std::to_wstring(entry.published) + L"/" +
                    std::to_wstring(entry.publishResets) +
                    L", render applied/fail/restore-fail=" +
                    std::to_wstring(entry.appliedCount) + L"/" +
                    std::to_wstring(entry.applyFailures) + L"/" +
                    std::to_wstring(entry.restoreFailures) + L".");
        }
    }
    log::Write(
        log::Level::info,
        L"Interpolation CPU probe: skeletal-capture total/mean=" +
            std::to_wstring(
                PerformanceMilliseconds(g_articulatedCaptureTicks)) +
            L" ms/" + std::to_wstring(PerformanceMicrosecondsPerCall(
                           g_articulatedCaptureTicks,
                           g_articulatedCaptureCalls)) +
            L" us; render apply total/mean=" +
            std::to_wstring(PerformanceMilliseconds(g_renderApplyTicks)) +
            L" ms/" + std::to_wstring(PerformanceMicrosecondsPerCall(
                           g_renderApplyTicks, g_renderApplyCalls)) +
            L" us; render restore total/mean=" +
            std::to_wstring(PerformanceMilliseconds(g_renderRestoreTicks)) +
            L" ms/" + std::to_wstring(PerformanceMicrosecondsPerCall(
                           g_renderRestoreTicks, g_renderRestoreCalls)) +
            L" us; skeletal bones written=" +
            std::to_wstring(g_articulatedBonesWritten) +
            L"; memory-region queries=" +
            std::to_wstring(g_accessQueries.exchange(
                0U, std::memory_order_relaxed)) +
            L"; agent mesh root published/reset/render-applied=" +
            std::to_wstring(g_agentRootPublished) + L"/" +
            std::to_wstring(g_agentRootResets) + L"/" +
            std::to_wstring(g_agentRootApplied) + L".");
    if (g_attachmentFrames != 0U) {
        log::Write(
            log::Level::info,
            L"Camera-to-skeleton attachment: matrix-camera frames/" +
                std::wstring(L"tick-pair searches/found/frames applied=") +
                std::to_wstring(g_attachmentFrames) + L"/" +
                std::to_wstring(g_attachmentSearches) + L"/" +
                std::to_wstring(g_attachmentFound) + L"/" +
                std::to_wstring(g_attachmentApplied) + L", last bone=" +
                (g_attachment.valid ? std::to_wstring(g_attachment.bone)
                                    : std::wstring(L"none")) +
                L", found/lost switches=" + std::to_wstring(g_attachmentToggles) +
                L", bone changes=" + std::to_wstring(g_attachmentBoneChanges) +
                L", stored-as-transpose=" +
                (g_attachment.transposed ? L"yes" : L"no") +
                L", worst accepted offset error=" +
                std::to_wstring(g_attachmentWorstError) + L".");
    }
    g_attachmentFrames = 0;
    g_attachmentSearches = 0;
    g_attachmentFound = 0;
    g_attachmentToggles = 0;
    g_attachmentBoneChanges = 0;
    g_attachmentApplied = 0;
    g_attachmentWorstError = 0.0F;
    g_agentRootPublished = 0;
    g_agentRootResets = 0;
    g_agentRootApplied = 0;
    g_articulatedBonesWritten = 0;
    g_handsCalls = 0;
    g_bodyCalls = 0;
    g_heldAttachmentCalls = 0;
    g_invalidMatrices = 0;
    g_rootApplied = 0;
    g_rootApplyFailures = 0;
    g_rootRestoreFailures = 0;
    g_readRootFailures = 0;
    g_collectNodeFailures = 0;
    g_inverseFailures = 0;
    g_multiplyFailures = 0;
    g_verifyFailures = 0;
    g_bodyBoneProbe.captures = 0;
    g_bodyBoneProbe.changedCaptures = 0;
    g_bodyBoneProbe.invalidCaptures = 0;
    g_handsBoneProbe.captures = 0;
    g_handsBoneProbe.changedCaptures = 0;
    g_handsBoneProbe.invalidCaptures = 0;
    g_heldApplied = 0;
    g_heldApplyFailures = 0;
    g_heldRestoreFailures = 0;
    g_heldRegistryOverflows = 0;
    g_scriptedRootCalls = 0;
    g_scriptedRootApplied = 0;
    g_scriptedRootApplyFailures = 0;
    g_scriptedRootRestoreFailures = 0;
    g_scriptedRootRegistryOverflows = 0;
    g_inspectionRootCalls = 0;
    g_inspectionApplied = 0;
    g_inspectionApplyFailures = 0;
    g_inspectionRestoreFailures = 0;
    g_inspectionRegistryOverflows = 0;
    g_worldPublished = 0;
    g_worldPublishResets = 0;
    g_worldApplied = 0;
    g_worldApplyFailures = 0;
    g_worldRestoreFailures = 0;
    g_articulatedRegistryOverflows = 0;
    g_articulatedCaptureTicks = 0;
    g_articulatedCaptureCalls = 0;
    g_renderApplyTicks = 0;
    g_renderApplyCalls = 0;
    g_renderRestoreTicks = 0;
    g_renderRestoreCalls = 0;
    g_posePublished = 0;
    g_posePublishResets = 0;
    g_poseApplied = 0;
    g_poseApplyFailures = 0;
    g_poseRestoreFailures = 0;
    for (ScriptedPoseEntry& entry : g_scriptedPoses) {
        entry.lookupCalls = 0;
        entry.setMatrixCalls = 0;
        entry.probe.captures = 0;
        entry.probe.changedCaptures = 0;
        entry.probe.invalidCaptures = 0;
        entry.published = 0;
        entry.publishResets = 0;
        entry.appliedCount = 0;
        entry.applyFailures = 0;
        entry.restoreFailures = 0;
    }
    for (ArticulatedPoseEntry& entry : g_articulatedPoses) {
        entry.probe.captures = 0;
        entry.probe.changedCaptures = 0;
        entry.probe.invalidCaptures = 0;
        entry.published = 0;
        entry.publishResets = 0;
        entry.discontinuityResets = 0;
        entry.appliedCount = 0;
        entry.applyFailures = 0;
        entry.restoreFailures = 0;
        entry.unchangedRenderSkips = 0;
    }
    g_lastReportMilliseconds = now;
}

bool ReadValidatedRoot(std::byte* const entity, root::Matrix& matrix) noexcept {
    if (!ValidEntityVtable(entity) || g_entityGetMatrix == nullptr) {
        return false;
    }
    root::Matrix returned{};
    root::TransformInfo info{};
    if (g_entityGetMatrix(entity, &returned) != &returned ||
        !root::Analyze(returned, info)) {
        return false;
    }
    matrix = returned;
    return true;
}

bool CollectNodes(std::byte* const entity, AppliedRoot& applied) noexcept {
    std::byte** begin = nullptr;
    std::byte** end = nullptr;
    if (!Read(entity + kEntityNodesBeginOffset, begin) ||
        !Read(entity + kEntityNodesEndOffset, end)) {
        return false;
    }
    if (begin == end) {
        std::byte* fallbackNode = nullptr;
        if (!Read(entity + kEntityFallbackNodeOffset, fallbackNode) ||
            fallbackNode == nullptr ||
            !IsAccessible(fallbackNode + kNodeLocalMatrixOffset,
                          sizeof(root::Matrix), true)) {
            return false;
        }
        applied.nodes[0] = fallbackNode;
        std::memcpy(&applied.matrices[0],
                    fallbackNode + kNodeLocalMatrixOffset,
                    sizeof(root::Matrix));
        applied.count = 1U;
        return true;
    }
    if (begin == nullptr || end < begin) {
        return false;
    }
    const auto byteCount = reinterpret_cast<std::uintptr_t>(end) -
                           reinterpret_cast<std::uintptr_t>(begin);
    if (byteCount % sizeof(std::byte*) != 0U) {
        return false;
    }
    const std::size_t count = byteCount / sizeof(std::byte*);
    if (count == 0U || count > kMaximumNodes ||
        !IsAccessible(begin, byteCount, false)) {
        return false;
    }
    std::byte* referenceNode = nullptr;
    static_cast<void>(Read(entity + kEntityReferenceNodeOffset, referenceNode));
    if (referenceNode == nullptr) {
        referenceNode = *begin;
    }
    bool referenceFound = false;
    for (std::size_t index = 0; index < count; ++index) {
        std::byte* const node = begin[index];
        if (!IsAccessible(node + kNodeLocalMatrixOffset, sizeof(root::Matrix),
                          true)) {
            return false;
        }
        applied.nodes[index] = node;
        std::memcpy(&applied.matrices[index], node + kNodeLocalMatrixOffset,
                    sizeof(root::Matrix));
        referenceFound = referenceFound || node == referenceNode;
    }
    if (!referenceFound) {
        return false;
    }
    applied.count = count;
    return true;
}

void SetNodeMatrix(std::byte* const node,
                   const root::Matrix& matrix) noexcept {
    std::memcpy(node + kNodeLocalMatrixOffset, &matrix, sizeof(matrix));
    g_nodeMatrixChanged(node, false);
}

bool Restore(AppliedRoot& applied) noexcept {
    if (!applied.active) {
        return true;
    }
    for (std::size_t index = 0; index < applied.count; ++index) {
        if (!IsAccessible(applied.nodes[index] + kNodeLocalMatrixOffset,
                          sizeof(root::Matrix), true)) {
            applied.active = false;
            return false;
        }
        SetNodeMatrix(applied.nodes[index], applied.matrices[index]);
    }
    root::Matrix restored{};
    const bool matches = ReadValidatedRoot(applied.entity, restored) &&
                         root::NearlyEqual(restored, applied.authoritative,
                                           2.0e-4F);
    applied.active = false;
    return matches;
}

bool Apply(TrackedRoot& tracked, const Kind expectedKind,
           const clock::Sample& sample, AppliedRoot& applied,
           const char* const expectedName = nullptr) noexcept {
    if (tracked.entity == nullptr) {
        return false;
    }
    // Sample first: a root without a moving endpoint pair needs no identity
    // read, and most registered roots are in that state on any given frame.
    root::SampleResult interpolated = tracked.history.Evaluate(sample);
    // Player-attached roots also follow the camera's mouse-look lead, which
    // moves them even when their own tick endpoints are identical.
    const bool followsView = g_viewYawLeadActive &&
        (expectedKind == Kind::hands || expectedKind == Kind::body ||
         expectedKind == Kind::heldAttachment);
    if (interpolated.kind != root::SampleKind::applied &&
        !(followsView && interpolated.kind == root::SampleKind::unchanged)) {
        return false;
    }
    std::string name;
    if (ReadKind(tracked.entity, name) != expectedKind ||
        (expectedName != nullptr && name != expectedName)) {
        Reset(tracked);
        return false;
    }
    if (followsView) {
        root::Matrix led{};
        root::TransformInfo ledInfo{};
        if (g_matrixMultiply(&led, &g_viewYawLead, &interpolated.matrix) !=
                &led ||
            !root::Analyze(led, ledInfo)) {
            ++g_multiplyFailures;
            ++g_rootApplyFailures;
            return false;
        }
        interpolated.matrix = led;
    }
    applied.active = false;
    applied.count = 0;
    applied.entity = tracked.entity;
    if (!ReadValidatedRoot(tracked.entity, applied.authoritative)) {
        ++g_readRootFailures;
        ++g_rootApplyFailures;
        tracked.history.Reset();
        return false;
    }
    if (!CollectNodes(tracked.entity, applied)) {
        ++g_collectNodeFailures;
        ++g_rootApplyFailures;
        tracked.history.Reset();
        return false;
    }

    root::Matrix inverse{};
    root::Matrix delta{};
    if (g_matrixInverse(&inverse, &applied.authoritative) != &inverse ||
        g_matrixMultiply(&delta, &interpolated.matrix, &inverse) != &delta) {
        ++g_inverseFailures;
        ++g_rootApplyFailures;
        tracked.history.Reset();
        return false;
    }
    applied.active = true;
    for (std::size_t index = 0; index < applied.count; ++index) {
        root::Matrix transformed{};
        if (g_matrixMultiply(&transformed, &delta,
                             &applied.matrices[index]) != &transformed) {
            ++g_multiplyFailures;
            ++g_rootApplyFailures;
            static_cast<void>(Restore(applied));
            tracked.history.Reset();
            return false;
        }
        SetNodeMatrix(applied.nodes[index], transformed);
    }
    root::Matrix actual{};
    if (!ReadValidatedRoot(tracked.entity, actual) ||
        !root::NearlyEqual(actual, interpolated.matrix, 2.0e-4F)) {
        ++g_verifyFailures;
        ++g_rootApplyFailures;
        static_cast<void>(Restore(applied));
        tracked.history.Reset();
        return false;
    }
    ++g_rootApplied;
    return true;
}

bool ReadWorldBodyMatrix(std::byte* const body,
                         const void* const expectedNewtonBody,
                         const char* const expectedName,
                         root::Matrix& matrix) noexcept {
    std::string name;
    const void* actualNewtonBody = nullptr;
    root::TransformInfo info{};
    return body != nullptr &&
           expectedNewtonBody != nullptr &&
           Read(body + kNewtonBodyPointerOffset, actualNewtonBody) &&
           actualNewtonBody == expectedNewtonBody &&
           ReadStringObject(body + 0x10U, name) &&
           expectedName != nullptr && name == expectedName &&
           IsAccessible(body + kNodeLocalMatrixOffset, sizeof(matrix), true) &&
           Read(body + kNodeLocalMatrixOffset, matrix) &&
           root::Analyze(matrix, info);
}

void SetWorldBodyMatrix(std::byte* const body,
                        const root::Matrix& matrix) noexcept {
    std::memcpy(body + kNodeLocalMatrixOffset, &matrix, sizeof(matrix));
    g_nodeMatrixChanged(body, true);
}

bool RestoreWorldBody(AppliedWorldBody& applied) noexcept {
    if (!applied.active) {
        return true;
    }
    root::Matrix current{};
    if (!ReadWorldBodyMatrix(applied.body, applied.newtonBody,
                             applied.name.data(), current)) {
        applied.active = false;
        return false;
    }
    SetWorldBodyMatrix(applied.body, applied.authoritative);
    root::Matrix restored{};
    const bool matches =
        ReadWorldBodyMatrix(applied.body, applied.newtonBody,
                            applied.name.data(), restored) &&
        root::NearlyEqual(restored, applied.authoritative, 2.0e-4F);
    applied.active = false;
    return matches;
}

bool ApplyWorldBody(const std::size_t index,
                    const clock::Sample& sample) noexcept {
    TrackedWorldBody& tracked = g_worldBodies[index];
    AppliedWorldBody& applied = g_appliedWorldBodies[index];
    if (!g_worldInterpolationEnabled || tracked.body == nullptr) {
        return false;
    }
    const root::SampleResult interpolated = tracked.history.Evaluate(sample);
    if (interpolated.kind != root::SampleKind::applied) {
        return false;
    }

    applied = {};
    applied.body = tracked.body;
    applied.newtonBody = tracked.newtonBody;
    applied.name = tracked.name;
    if (!ReadWorldBodyMatrix(tracked.body, tracked.newtonBody,
                             tracked.name.data(),
                             applied.authoritative)) {
        ++g_worldApplyFailures;
        tracked.body = nullptr;
        tracked.history.Reset();
        return false;
    }
    applied.active = true;
    SetWorldBodyMatrix(tracked.body, interpolated.matrix);
    root::Matrix actual{};
    if (!ReadWorldBodyMatrix(tracked.body, tracked.newtonBody,
                             tracked.name.data(), actual) ||
        !root::NearlyEqual(actual, interpolated.matrix, 2.0e-4F)) {
        ++g_worldApplyFailures;
        static_cast<void>(RestoreWorldBody(applied));
        tracked.history.Reset();
        return false;
    }
    ++g_worldApplied;
    return true;
}

void RegisterScriptedPose(ScriptedPoseEntry& entry,
                          std::byte* const entity) noexcept {
    if (entry.entity != entity) {
        ResetScriptedPose(entry, true);
        entry.entity = entity;
    }
    if (!entry.registrationLogged) {
        log::Write(log::Level::info,
                   std::wstring(entry.label) +
                       L" scripted entity registered for guarded skeletal "
                       L"pose interpolation.");
        entry.registrationLogged = true;
    }
}

std::byte** __fastcall EntityNameResolverHook(
    const void* const entityName, const void* const apiName,
    const std::uint32_t entityType) noexcept {
    std::string requestedName;
    ScriptedPoseEntry* requested = nullptr;
    if (ReadStringObject(static_cast<const std::byte*>(entityName),
                         requestedName)) {
        requested = FindScriptedPose(requestedName);
        if (requested != nullptr) {
            ++requested->lookupCalls;
        }
    }

    std::byte** const result =
        g_originalEntityNameResolver(entityName, apiName, entityType);
    if (requested == nullptr || result == nullptr) {
        return result;
    }

    std::byte** begin = nullptr;
    std::byte** end = nullptr;
    if (!Read(result, begin) ||
        !Read(reinterpret_cast<const std::byte*>(result) + sizeof(begin), end) ||
        begin == nullptr || end < begin) {
        return result;
    }
    const auto byteCount = reinterpret_cast<std::uintptr_t>(end) -
                           reinterpret_cast<std::uintptr_t>(begin);
    constexpr std::size_t maximumResolvedEntities = 64U;
    if (byteCount == 0U || byteCount % sizeof(std::byte*) != 0U ||
        byteCount > maximumResolvedEntities * sizeof(std::byte*) ||
        !IsAccessible(begin, byteCount, false)) {
        return result;
    }
    for (std::byte** current = begin; current != end; ++current) {
        std::byte* entity = nullptr;
        if (!Read(current, entity)) {
            continue;
        }
        std::string actualName;
        if (ReadKind(entity, actualName) == Kind::scriptedPose &&
            actualName == requestedName) {
            RegisterScriptedPose(*requested, entity);
            break;
        }
    }
    return result;
}

void __fastcall EntitySetMatrixHook(std::byte* const entity,
                                    const root::Matrix* const matrix) noexcept {
    const std::uintptr_t callerRva = FunctionRva(_ReturnAddress());
    void** vtable = nullptr;
    const std::uintptr_t vtableRva =
        Read(entity, vtable) ? FunctionRva(vtable) : 0U;
    g_originalEntitySetMatrix(entity, matrix);

    std::string name;
    const Kind kind = ReadKind(entity, name);
    if (g_diagnosticsEnabled && !name.empty()) {
        ObserveEntityRoot(entity, name, matrix, callerRva, vtableRva);
    }
    ScriptedPoseEntry* const scripted = kind == Kind::scriptedPose
        ? FindScriptedPose(name)
        : nullptr;
    if (kind == Kind::hands) {
        ++g_handsCalls;
    } else if (kind == Kind::body) {
        ++g_bodyCalls;
    } else if (kind == Kind::heldAttachment) {
        ++g_heldAttachmentCalls;
    } else if (kind == Kind::scriptedRoot) {
        ++g_scriptedRootCalls;
    } else if (scripted != nullptr) {
        ++scripted->setMatrixCalls;
    } else if (kind == Kind::inspectionRoot) {
        ++g_inspectionRootCalls;
    }
    root::TransformInfo info{};
    root::Matrix copy{};
    if (!Read(matrix, copy) || !root::Analyze(copy, info) ||
        !ValidEntityVtable(entity)) {
        ++g_invalidMatrices;
        if (kind == Kind::hands) {
            Reset(g_hands);
        } else if (kind == Kind::body) {
            Reset(g_body);
        } else if (kind == Kind::heldAttachment) {
            ResetHeldAttachment(entity);
        } else if (kind == Kind::scriptedRoot) {
            ResetScriptedRoot(entity);
        } else if (scripted != nullptr) {
            ResetScriptedPose(*scripted, true);
        } else if (kind == Kind::inspectionRoot) {
            ResetInspectionRoot(entity);
        }
    } else {
        if (kind == Kind::scriptedPose && scripted != nullptr) {
            RegisterScriptedPose(*scripted, entity);
        } else if (kind != Kind::inspectionRoot && kind != Kind::other) {
            LogDiscovery(kind, name, entity, copy, info);
        }
        if (g_interpolationEnabled && kind != Kind::scriptedPose) {
            if (kind == Kind::heldAttachment) {
                StageHeldAttachment(entity, copy, name);
            } else if (kind == Kind::scriptedRoot) {
                StageScriptedRoot(entity, copy, name, callerRva, vtableRva);
            } else if (kind == Kind::inspectionRoot) {
                StageInspectionRoot(entity, copy, name, callerRva, vtableRva);
            } else if (kind == Kind::other && !name.empty()) {
                ++g_scriptedRootCalls;
                StageScriptedRoot(entity, copy, name, callerRva, vtableRva);
            } else {
                Stage(kind == Kind::hands ? g_hands : g_body, entity, copy);
            }
        }
    }
    MaybeReport();
}

void __fastcall EntitySetVarBoolHook(
    std::byte* const entity, const std::byte* const variableName,
    const bool value) noexcept {
    std::string name;
    const bool isReadableMarker = value &&
        ReadStringObject(variableName, name) && name == "IsHUDVersion";
    g_originalEntitySetVarBool(entity, variableName, value);
    if (g_interpolationEnabled && isReadableMarker) {
        RegisterInspectionIdentity(entity);
    }
}

void __fastcall NewtonTransformCallbackHook(const void* const newtonBody,
                                            const float* const matrix,
                                            const int threadIndex) noexcept {
    g_originalNewtonTransformCallback(newtonBody, matrix, threadIndex);
    if ((!g_diagnosticsEnabled && !g_worldInterpolationEnabled) ||
        g_newtonBodyGetUserData == nullptr) {
        return;
    }
    ObserveWorldBody(
        static_cast<std::byte*>(g_newtonBodyGetUserData(newtonBody)),
        newtonBody);
}

void __fastcall LuxPropOnUpdateHook(std::byte* const prop,
                                    const float timeStep) noexcept {
    g_originalLuxPropOnUpdate(prop, timeStep);
    StageArticulatedProp(prop);
}

void __fastcall LuxAgentOnUpdateHook(std::byte* const agent,
                                     const float timeStep) noexcept {
    g_originalLuxAgentOnUpdate(agent, timeStep);
    StageArticulatedProp(agent, true);
}

std::wstring MinHookError(const wchar_t* const operation,
                          const MH_STATUS status) {
    const char* statusName = MH_StatusToString(status);
    std::wstring converted;
    if (statusName != nullptr) {
        while (*statusName != '\0') {
            converted.push_back(static_cast<wchar_t>(*statusName));
            ++statusName;
        }
    }
    return std::wstring(operation) + L" failed: " + converted + L" (" +
           std::to_wstring(static_cast<int>(status)) + L").";
}

} // namespace

bool Start(const HMODULE executable, const bool rootInterpolationEnabled,
           const bool poseInterpolationEnabled,
           const bool worldInterpolationEnabled,
           const bool ragdollPoseInterpolationEnabled,
           const bool diagnosticsEnabled, std::wstring& error) noexcept {
    if (executable == nullptr) {
        error = L"Host module is null while installing the player-root runtime.";
        return false;
    }
    g_executableBase = reinterpret_cast<std::byte*>(executable);
    std::byte* const hookTarget = g_executableBase + kEntitySetMatrixRva;
    std::byte* const setVarBoolTarget =
        g_executableBase + kEntitySetVarBoolRva;
    std::byte* const newtonTransformTarget =
        g_executableBase + kNewtonTransformCallbackRva;
    std::byte* const luxPropOnUpdateTarget =
        g_executableBase + kLuxPropOnUpdateRva;
    const bool newtonRuntimeEnabled =
        diagnosticsEnabled || worldInterpolationEnabled;
    if (!Matches(hookTarget, kEntitySetMatrixEntry) ||
        (rootInterpolationEnabled &&
         !Matches(setVarBoolTarget, kEntitySetVarBoolEntry)) ||
        !Matches(g_executableBase + kEntityGetMatrixRva,
                 kEntityGetMatrixEntry) ||
        !Matches(g_executableBase + kEntityVirtualSetMatrixRva,
                 kEntityVirtualSetMatrixEntry) ||
        !Matches(g_executableBase + kMatrixInverseRva, kMatrixInverseEntry) ||
        !Matches(g_executableBase + kMatrixMultiplyRva, kMatrixMultiplyEntry) ||
        !Matches(g_executableBase + kNodeMatrixChangedRva,
                 kNodeMatrixChangedEntry) ||
        (newtonRuntimeEnabled &&
         !Matches(newtonTransformTarget, kNewtonTransformCallbackEntry)) ||
        (ragdollPoseInterpolationEnabled &&
         !Matches(luxPropOnUpdateTarget, kLuxPropOnUpdateEntry)) ||
        ((poseInterpolationEnabled || ragdollPoseInterpolationEnabled) &&
         !Matches(g_executableBase + kBoneSetMatrixRva,
                  kBoneSetMatrixEntry)) ||
        (poseInterpolationEnabled &&
         !Matches(g_executableBase + kEntityNameResolverRva,
                  kEntityNameResolverEntry))) {
        error = L"Player-root native entry bytes do not match the supported build.";
        g_executableBase = nullptr;
        return false;
    }
    g_entityGetMatrix = reinterpret_cast<EntityGetMatrix>(
        g_executableBase + kEntityGetMatrixRva);
    g_matrixInverse = reinterpret_cast<MatrixInverse>(
        g_executableBase + kMatrixInverseRva);
    g_matrixMultiply = reinterpret_cast<MatrixMultiply>(
        g_executableBase + kMatrixMultiplyRva);
    g_nodeMatrixChanged = reinterpret_cast<NodeMatrixChanged>(
        g_executableBase + kNodeMatrixChangedRva);
    g_boneSetMatrix = reinterpret_cast<BoneSetMatrix>(
        g_executableBase + kBoneSetMatrixRva);
    g_interpolationEnabled = rootInterpolationEnabled;
    g_poseInterpolationEnabled = poseInterpolationEnabled;
    g_worldInterpolationEnabled = worldInterpolationEnabled;
    g_articulatedPoseInterpolationEnabled =
        ragdollPoseInterpolationEnabled;
    g_diagnosticsEnabled = diagnosticsEnabled;
    g_performanceFrequency = {};
    if (g_diagnosticsEnabled) {
        static_cast<void>(QueryPerformanceFrequency(&g_performanceFrequency));
    }
    if (newtonRuntimeEnabled) {
        const HMODULE newton = GetModuleHandleW(L"Newton.dll");
        g_newtonBodyGetUserData = newton == nullptr
            ? nullptr
            : reinterpret_cast<NewtonBodyGetUserData>(
                  GetProcAddress(newton, "NewtonBodyGetUserData"));
        if (g_newtonBodyGetUserData == nullptr) {
            error = L"NewtonBodyGetUserData was unavailable for the guarded "
                    L"world-body probe.";
            g_executableBase = nullptr;
            return false;
        }
    }

    MH_STATUS status = MH_Initialize();
    if (status != MH_OK && status != MH_ERROR_ALREADY_INITIALIZED) {
        error = MinHookError(L"MH_Initialize", status);
        return false;
    }
    status = MH_CreateHook(hookTarget,
                           reinterpret_cast<void*>(&EntitySetMatrixHook),
                           reinterpret_cast<void**>(&g_originalEntitySetMatrix));
    if (status != MH_OK) {
        error = MinHookError(L"MH_CreateHook(cLuxEntity::SetMatrix)", status);
        return false;
    }
    std::byte* const resolverTarget =
        g_executableBase + kEntityNameResolverRva;
    if (poseInterpolationEnabled) {
        status = MH_CreateHook(
            resolverTarget, reinterpret_cast<void*>(&EntityNameResolverHook),
            reinterpret_cast<void**>(&g_originalEntityNameResolver));
        if (status != MH_OK) {
            static_cast<void>(MH_RemoveHook(hookTarget));
            g_originalEntitySetMatrix = nullptr;
            error = MinHookError(L"MH_CreateHook(entity-name resolver)",
                                 status);
            return false;
        }
    }
    if (newtonRuntimeEnabled) {
        status = MH_CreateHook(
            newtonTransformTarget,
            reinterpret_cast<void*>(&NewtonTransformCallbackHook),
            reinterpret_cast<void**>(&g_originalNewtonTransformCallback));
        if (status != MH_OK) {
            if (poseInterpolationEnabled) {
                static_cast<void>(MH_RemoveHook(resolverTarget));
                g_originalEntityNameResolver = nullptr;
            }
            static_cast<void>(MH_RemoveHook(hookTarget));
            g_originalEntitySetMatrix = nullptr;
            g_newtonBodyGetUserData = nullptr;
            error = MinHookError(L"MH_CreateHook(Newton transform callback)",
                                 status);
            return false;
        }
        status = MH_EnableHook(newtonTransformTarget);
        if (status != MH_OK) {
            static_cast<void>(MH_RemoveHook(newtonTransformTarget));
            g_originalNewtonTransformCallback = nullptr;
            if (poseInterpolationEnabled) {
                static_cast<void>(MH_RemoveHook(resolverTarget));
                g_originalEntityNameResolver = nullptr;
            }
            static_cast<void>(MH_RemoveHook(hookTarget));
            g_originalEntitySetMatrix = nullptr;
            g_newtonBodyGetUserData = nullptr;
            error = MinHookError(L"MH_EnableHook(Newton transform callback)",
                                 status);
            return false;
        }
    }
    status = MH_EnableHook(hookTarget);
    if (status != MH_OK) {
        if (newtonRuntimeEnabled) {
            static_cast<void>(MH_DisableHook(newtonTransformTarget));
            static_cast<void>(MH_RemoveHook(newtonTransformTarget));
            g_originalNewtonTransformCallback = nullptr;
            g_newtonBodyGetUserData = nullptr;
        }
        if (poseInterpolationEnabled) {
            static_cast<void>(MH_RemoveHook(resolverTarget));
            g_originalEntityNameResolver = nullptr;
        }
        static_cast<void>(MH_RemoveHook(hookTarget));
        g_originalEntitySetMatrix = nullptr;
        error = MinHookError(L"MH_EnableHook(cLuxEntity::SetMatrix)", status);
        return false;
    }
    if (poseInterpolationEnabled) {
        status = MH_EnableHook(resolverTarget);
        if (status != MH_OK) {
            if (newtonRuntimeEnabled) {
                static_cast<void>(MH_DisableHook(newtonTransformTarget));
                static_cast<void>(MH_RemoveHook(newtonTransformTarget));
                g_originalNewtonTransformCallback = nullptr;
                g_newtonBodyGetUserData = nullptr;
            }
            static_cast<void>(MH_DisableHook(hookTarget));
            static_cast<void>(MH_RemoveHook(hookTarget));
            static_cast<void>(MH_RemoveHook(resolverTarget));
            g_originalEntitySetMatrix = nullptr;
            g_originalEntityNameResolver = nullptr;
            error = MinHookError(L"MH_EnableHook(entity-name resolver)",
                                 status);
            return false;
        }
    }
    if (rootInterpolationEnabled) {
        status = MH_CreateHook(
            setVarBoolTarget,
            reinterpret_cast<void*>(&EntitySetVarBoolHook),
            reinterpret_cast<void**>(&g_originalEntitySetVarBool));
        if (status != MH_OK) {
            error = MinHookError(L"MH_CreateHook(cLuxEntity::SetVarBool)",
                                 status);
            Stop();
            return false;
        }
        status = MH_EnableHook(setVarBoolTarget);
        if (status != MH_OK) {
            error = MinHookError(L"MH_EnableHook(cLuxEntity::SetVarBool)",
                                 status);
            Stop();
            return false;
        }
    }
    if (ragdollPoseInterpolationEnabled) {
        status = MH_CreateHook(
            luxPropOnUpdateTarget,
            reinterpret_cast<void*>(&LuxPropOnUpdateHook),
            reinterpret_cast<void**>(&g_originalLuxPropOnUpdate));
        if (status != MH_OK) {
            error = MinHookError(L"MH_CreateHook(cLuxProp::OnUpdate)", status);
            Stop();
            return false;
        }
        status = MH_EnableHook(luxPropOnUpdateTarget);
        if (status != MH_OK) {
            error = MinHookError(L"MH_EnableHook(cLuxProp::OnUpdate)", status);
            Stop();
            return false;
        }
        // Agents share the skeletal path. A mismatch here only leaves AI
        // characters at the fixed rate; props keep working.
        std::byte* const agentTarget =
            g_executableBase + kLuxAgentOnUpdateRva;
        if (!Matches(agentTarget, kLuxAgentOnUpdateEntry) ||
            !Matches(g_executableBase + kLuxAgentGetMeshRva,
                     kLuxAgentGetMeshEntry)) {
            log::Write(log::Level::warning,
                       L"cLuxAgent entry bytes do not match the supported "
                       L"build; agent interpolation stays off.");
        } else if (MH_CreateHook(
                       agentTarget,
                       reinterpret_cast<void*>(&LuxAgentOnUpdateHook),
                       reinterpret_cast<void**>(
                           &g_originalLuxAgentOnUpdate)) != MH_OK ||
                   MH_EnableHook(agentTarget) != MH_OK) {
            static_cast<void>(MH_RemoveHook(agentTarget));
            g_originalLuxAgentOnUpdate = nullptr;
            log::Write(log::Level::warning,
                       L"cLuxAgent::OnUpdate could not be detoured; agent "
                       L"interpolation stays off.");
        }
    }
    return true;
}

void Commit(const std::uint64_t tickSerial,
            const std::uint64_t clockGeneration) noexcept {
    InvalidateAccessRegions();
    if (g_worldInterpolationEnabled) {
        std::array<StagedWorldBody, kMaximumInterpolatedWorldBodies> staged{};
        while (g_worldStageLock.test_and_set(std::memory_order_acquire)) {
            SwitchToThread();
        }
        staged = g_stagedWorldBodies;
        for (StagedWorldBody& entry : g_stagedWorldBodies) {
            entry.available = false;
        }
        g_worldStageLock.clear(std::memory_order_release);
        std::array<bool, kMaximumInterpolatedWorldBodies> seen{};
        for (const StagedWorldBody& publishedBody : staged) {
            if (!publishedBody.available) {
                continue;
            }
            std::size_t trackedIndex = g_worldBodies.size();
            std::size_t freeIndex = g_worldBodies.size();
            std::size_t staleIndex = g_worldBodies.size();
            for (std::size_t index = 0; index < g_worldBodies.size();
                 ++index) {
                const TrackedWorldBody& candidate = g_worldBodies[index];
                if (candidate.body == publishedBody.body &&
                    candidate.newtonBody == publishedBody.newtonBody &&
                    std::strcmp(candidate.name.data(),
                                publishedBody.name.data()) == 0) {
                    trackedIndex = index;
                    break;
                }
                if (candidate.body == nullptr &&
                    freeIndex == g_worldBodies.size()) {
                    freeIndex = index;
                } else if (candidate.body != nullptr &&
                           candidate.lastSeenSerial + 1U < tickSerial &&
                           staleIndex == g_worldBodies.size()) {
                    staleIndex = index;
                }
            }
            if (trackedIndex == g_worldBodies.size()) {
                trackedIndex = freeIndex != g_worldBodies.size()
                    ? freeIndex
                    : staleIndex;
                if (trackedIndex == g_worldBodies.size()) {
                    ++g_worldStageOverflows;
                    continue;
                }
                g_worldBodies[trackedIndex] = {};
                g_worldBodies[trackedIndex].body = publishedBody.body;
                g_worldBodies[trackedIndex].newtonBody =
                    publishedBody.newtonBody;
                g_worldBodies[trackedIndex].name = publishedBody.name;
            }
            TrackedWorldBody& tracked = g_worldBodies[trackedIndex];
            tracked.lastSeenSerial = tickSerial;
            seen[trackedIndex] = true;
            const root::PublishKind published = tracked.history.Publish(
                tickSerial, clockGeneration, publishedBody.matrix);
            ++g_worldPublished;
            if (published != root::PublishKind::appended &&
                published != root::PublishKind::unchanged) {
                ++g_worldPublishResets;
            }
            if (published == root::PublishKind::invalid) {
                tracked = {};
            }
        }
        for (std::size_t index = 0; index < g_worldBodies.size(); ++index) {
            if (g_worldBodies[index].body != nullptr && !seen[index] &&
                g_worldBodies[index].lastSeenSerial < tickSerial) {
                g_worldBodies[index] = {};
            }
        }
    }
    if (g_articulatedPoseInterpolationEnabled) {
        const std::int64_t started = PerformanceCounterNow();
        for (ArticulatedPoseEntry& entry : g_articulatedPoses) {
            if (entry.entity == nullptr) {
                continue;
            }
            const bool stagedThisTick = entry.staged;
            entry.staged = false;
            BoneCapture articulatedCapture{};
            if (CaptureBones(entry.entity, entry.probe,
                             entry.agent ? L"Agent" : L"Skeletal prop",
                             articulatedCapture, entry.meshOffset)) {
                entry.lastSeenSerial = tickSerial;
                PublishArticulatedPose(entry, articulatedCapture, tickSerial,
                                       clockGeneration);
                if (entry.agent) {
                    root::Matrix meshMatrix{};
                    const root::PublishKind rootPublished =
                        Read(articulatedCapture.mesh + kNodeLocalMatrixOffset,
                             meshMatrix)
                            ? entry.meshRoot.Publish(tickSerial,
                                                     clockGeneration,
                                                     meshMatrix)
                            : root::PublishKind::invalid;
                    ++g_agentRootPublished;
                    if (rootPublished == root::PublishKind::invalid) {
                        entry.meshRoot.Reset();
                    }
                    if (rootPublished != root::PublishKind::appended &&
                        rootPublished != root::PublishKind::unchanged) {
                        ++g_agentRootResets;
                    }
                }
            } else if (!stagedThisTick) {
                ClearArticulatedPose(entry);
            } else {
                entry.pose.history.Reset();
            }
        }
        AccumulatePerformance(started, g_articulatedCaptureTicks,
                              g_articulatedCaptureCalls);
    }
    BoneCapture capture{};
    if (CaptureBones(g_body.entity, g_bodyBoneProbe, L"Body", capture)) {
        PublishBodyPose(capture, tickSerial, clockGeneration);
    } else if (g_poseInterpolationEnabled) {
        ResetBodyPose();
    }
    BoneCapture handsCapture{};
    static_cast<void>(
        CaptureBones(g_hands.entity, g_handsBoneProbe, L"Hands", handsCapture));
    for (ScriptedPoseEntry& entry : g_scriptedPoses) {
        BoneCapture scriptedCapture{};
        if (CaptureBones(entry.entity, entry.probe, entry.label,
                         scriptedCapture)) {
            PublishScriptedPose(entry, scriptedCapture, tickSerial,
                                clockGeneration);
        } else if (g_poseInterpolationEnabled) {
            entry.pose.history.Reset();
        }
    }
    if (!g_interpolationEnabled) {
        return;
    }
    const auto commitOne = [tickSerial, clockGeneration](TrackedRoot& tracked) {
        if (!tracked.hasStaged) {
            return;
        }
        const root::PublishKind result = tracked.history.Publish(
            tickSerial, clockGeneration, tracked.staged);
        tracked.hasStaged = false;
        if (result == root::PublishKind::invalid) {
            tracked.history.Reset();
        }
    };
    commitOne(g_hands);
    commitOne(g_body);
    for (TrackedRoot& tracked : g_heldAttachments) {
        commitOne(tracked);
    }
    for (TrackedNamedRoot& entry : g_scriptedRoots) {
        if (entry.root.hasStaged) {
            commitOne(entry.root);
        } else if (entry.root.entity != nullptr) {
            ResetScriptedRoot(entry);
        }
    }
    for (TrackedNamedRoot& entry : g_inspectionRoots) {
        commitOne(entry.root);
    }
}

void ResetHistory() noexcept {
    g_hands.history.Reset();
    g_body.history.Reset();
    g_hands.hasStaged = false;
    g_body.hasStaged = false;
    for (TrackedRoot& tracked : g_heldAttachments) {
        tracked.history.Reset();
        tracked.hasStaged = false;
    }
    for (TrackedNamedRoot& entry : g_scriptedRoots) {
        entry.root.history.Reset();
        entry.root.hasStaged = false;
    }
    for (TrackedNamedRoot& entry : g_inspectionRoots) {
        entry.root.history.Reset();
        entry.root.hasStaged = false;
    }
    g_bodyPose.history.Reset();
    for (ScriptedPoseEntry& entry : g_scriptedPoses) {
        entry.pose.history.Reset();
    }
    for (TrackedWorldBody& tracked : g_worldBodies) {
        tracked.body = nullptr;
        tracked.history.Reset();
    }
    for (ArticulatedPoseEntry& entry : g_articulatedPoses) {
        entry.pose.history.Reset();
        entry.meshRoot.Reset();
        entry.staged = false;
        entry.interpolateCurrentInterval = false;
        entry.lastSeenSerial = 0;
    }
    while (g_worldStageLock.test_and_set(std::memory_order_acquire)) {
        SwitchToThread();
    }
    g_stagedWorldBodies = {};
    g_worldStageLock.clear(std::memory_order_release);
}

bool SolveAttachedCamera(const clock::Sample& sample,
                         const attach::CameraEndpoint& older,
                         const attach::CameraEndpoint& newer,
                         attach::CameraEndpoint& presented) noexcept {
    if (!g_poseInterpolationEnabled || !g_bodyParentsValid ||
        g_body.entity == nullptr || g_bodyPose.count == 0U) {
        return false;
    }
    const pose::Pose* olderPose = nullptr;
    const pose::Pose* newerPose = nullptr;
    if (!g_bodyPose.history.Endpoints(sample, olderPose, newerPose) ||
        newerPose->count != g_bodyPose.count) {
        return false;
    }
    root::Matrix olderRoot{};
    root::Matrix newerRoot{};
    const bool rootHistory =
        g_body.history.Endpoints(sample, olderRoot, newerRoot);
    if (!rootHistory) {
        // A root with no stored pair has not been republished; its current
        // authoritative transform then stands for both ticks.
        if (!ReadValidatedRoot(g_body.entity, newerRoot)) {
            return false;
        }
        olderRoot = newerRoot;
    }
    ++g_attachmentFrames;
    if (g_attachmentSerial != sample.newerSerial ||
        g_attachmentGeneration != sample.generation) {
        g_attachmentSerial = sample.newerSerial;
        g_attachmentGeneration = sample.generation;
        const attach::Attachment previous = g_attachment;
        g_attachment = attach::FindAttachment(
            olderPose->matrices.data(), newerPose->matrices.data(),
            g_bodyParents.data(), g_bodyPose.count, olderRoot, newerRoot,
            older, newer, kAttachmentTolerance, previous);
        ++g_attachmentSearches;
        // Each change of placement method or bone between ticks is a point
        // where the presented camera can jump.
        if (previous.valid != g_attachment.valid) {
            ++g_attachmentToggles;
        } else if (previous.valid && previous.bone != g_attachment.bone) {
            ++g_attachmentBoneChanges;
        }
        if (g_attachment.valid) {
            ++g_attachmentFound;
            g_attachmentWorstError =
                std::max(g_attachmentWorstError, g_attachment.error);
        }
    }
    if (!g_attachment.valid) {
        return false;
    }

    // The same samples the body root and pose substitution use this frame.
    root::Matrix rootNow = newerRoot;
    if (rootHistory) {
        const root::SampleResult rootSample = g_body.history.Evaluate(sample);
        if (rootSample.kind != root::SampleKind::applied &&
            rootSample.kind != root::SampleKind::unchanged) {
            return false;
        }
        rootNow = rootSample.matrix;
    }
    const pose::SampleResult poseSample = g_bodyPose.history.Evaluate(sample);
    const root::Matrix* locals = nullptr;
    if (poseSample.kind == pose::SampleKind::applied) {
        locals = poseSample.pose.matrices.data();
    } else if (poseSample.kind == pose::SampleKind::unchanged) {
        locals = newerPose->matrices.data();
    } else {
        return false;
    }
    root::Matrix model{};
    if (!attach::ComposeBone(locals, g_bodyParents.data(), g_bodyPose.count,
                             g_attachment.bone, model) ||
        !attach::PresentCamera(g_attachment,
                               attach::Multiply(rootNow, model), sample.alpha,
                               presented)) {
        return false;
    }
    ++g_attachmentApplied;
    return true;
}

void SetViewYawLead(const float yawRadians, const float pivotX,
                    const float pivotY, const float pivotZ) noexcept {
    g_viewYawLeadActive = false;
    if (!std::isfinite(yawRadians) || std::abs(yawRadians) < 1.0e-7F ||
        !std::isfinite(pivotX) || !std::isfinite(pivotY) ||
        !std::isfinite(pivotZ)) {
        return;
    }
    // T(pivot) * Ry(yaw) * T(-pivot), row-major with translation in the
    // fourth column, matching the engine's camera yaw about world +Y.
    const float cosine = std::cos(yawRadians);
    const float sine = std::sin(yawRadians);
    g_viewYawLead = root::Identity();
    g_viewYawLead.values[0] = cosine;
    g_viewYawLead.values[2] = sine;
    g_viewYawLead.values[8] = -sine;
    g_viewYawLead.values[10] = cosine;
    g_viewYawLead.values[3] = pivotX - cosine * pivotX - sine * pivotZ;
    g_viewYawLead.values[7] = 0.0F;
    g_viewYawLead.values[11] = pivotZ + sine * pivotX - cosine * pivotZ;
    static_cast<void>(pivotY);
    g_viewYawLeadActive = true;
}

bool BeginRender(const clock::Sample& sample) noexcept {
    if ((!g_interpolationEnabled && !g_poseInterpolationEnabled &&
         !g_articulatedPoseInterpolationEnabled) ||
        g_renderActive) {
        return false;
    }
    const std::int64_t started = PerformanceCounterNow();
    InvalidateAccessRegions();
    g_renderActive = true;
    const bool handsApplied = g_interpolationEnabled &&
        Apply(g_hands, Kind::hands, sample, g_appliedHands);
    const bool bodyApplied = g_interpolationEnabled &&
        Apply(g_body, Kind::body, sample, g_appliedBody);
    bool heldApplied = false;
    if (g_interpolationEnabled) {
        for (std::size_t index = 0; index < g_heldAttachments.size();
             ++index) {
            const std::uint64_t failuresBefore = g_rootApplyFailures;
            if (Apply(g_heldAttachments[index], Kind::heldAttachment, sample,
                      g_appliedHeldAttachments[index])) {
                ++g_heldApplied;
                heldApplied = true;
            } else if (g_rootApplyFailures != failuresBefore) {
                ++g_heldApplyFailures;
            }
        }
    }
    bool scriptedRootApplied = false;
    if (g_interpolationEnabled) {
        for (std::size_t index = 0; index < g_scriptedRoots.size(); ++index) {
            const std::uint64_t failuresBefore = g_rootApplyFailures;
            TrackedNamedRoot& entry = g_scriptedRoots[index];
            if (Apply(entry.root, Kind::scriptedRoot, sample,
                      g_appliedScriptedRoots[index], entry.name.data())) {
                ++g_scriptedRootApplied;
                scriptedRootApplied = true;
            } else if (g_rootApplyFailures != failuresBefore) {
                ++g_scriptedRootApplyFailures;
            }
        }
    }
    bool inspectionApplied = false;
    if (g_interpolationEnabled) {
        for (std::size_t index = 0; index < g_inspectionRoots.size();
             ++index) {
            const std::uint64_t failuresBefore = g_rootApplyFailures;
            TrackedNamedRoot& entry = g_inspectionRoots[index];
            if (Apply(entry.root, Kind::inspectionRoot, sample,
                      g_appliedInspectionRoots[index], entry.name.data())) {
                ++g_inspectionApplied;
                inspectionApplied = true;
            } else if (g_rootApplyFailures != failuresBefore) {
                ++g_inspectionApplyFailures;
            }
        }
    }
    if ((handsApplied || bodyApplied) &&
        !g_loggedInterpolation.exchange(true, std::memory_order_acq_rel)) {
        log::Write(log::Level::info,
                   L"Player hands/body root interpolation active on the "
                   L"camera presentation clock.");
    }
    if (heldApplied &&
        !g_loggedHeldInterpolation.exchange(true,
                                             std::memory_order_acq_rel)) {
        log::Write(log::Level::info,
                   L"Held/accessory root interpolation active on the camera "
                   L"presentation clock.");
    }
    if (scriptedRootApplied &&
        !g_loggedScriptedRootInterpolation.exchange(
            true, std::memory_order_acq_rel)) {
        log::Write(log::Level::info,
                   L"Scripted entity-root interpolation active on the "
                   L"camera presentation clock.");
    }
    if (inspectionApplied &&
        !g_loggedInspectionInterpolation.exchange(
            true, std::memory_order_acq_rel)) {
        log::Write(log::Level::info,
                   L"Inspection entity-root interpolation active on the "
                   L"camera presentation clock.");
    }
    bool worldApplied = false;
    for (std::size_t index = 0; index < g_worldBodies.size(); ++index) {
        worldApplied = ApplyWorldBody(index, sample) || worldApplied;
    }
    if (worldApplied &&
        !g_loggedWorldInterpolation.exchange(true,
                                              std::memory_order_acq_rel)) {
        log::Write(log::Level::info,
                   L"Bounded Newton rigid-body world-root interpolation "
                   L"active on the camera presentation clock.");
    }
    const bool poseApplied = ApplyBodyPose(sample);
    if (poseApplied &&
        !g_loggedPoseInterpolation.exchange(true,
                                             std::memory_order_acq_rel)) {
        log::Write(log::Level::info,
                   L"PlayerBody skeletal-pose interpolation active on the "
                   L"camera presentation clock.");
    }
    bool scriptedPoseApplied = false;
    for (ScriptedPoseEntry& entry : g_scriptedPoses) {
        scriptedPoseApplied = ApplyScriptedPose(entry, sample) ||
                              scriptedPoseApplied;
    }
    bool articulatedPoseApplied = false;
    if (g_articulatedPoseInterpolationEnabled) {
        for (ArticulatedPoseEntry& entry : g_articulatedPoses) {
            if (entry.entity != nullptr) {
                articulatedPoseApplied = ApplyArticulatedPose(entry, sample) ||
                                          articulatedPoseApplied;
            }
        }
    }
    if (articulatedPoseApplied &&
        !g_loggedArticulatedPoseInterpolation.exchange(
            true, std::memory_order_acq_rel)) {
        log::Write(log::Level::info,
                   L"Generic skeletal prop pose interpolation "
                   L"active on the camera presentation clock.");
    }
    const bool applied = handsApplied || bodyApplied || heldApplied ||
        scriptedRootApplied || inspectionApplied || worldApplied ||
        poseApplied || scriptedPoseApplied || articulatedPoseApplied;
    AccumulatePerformance(started, g_renderApplyTicks, g_renderApplyCalls);
    return applied;
}

void EndRender() noexcept {
    if (!g_renderActive) {
        return;
    }
    const std::int64_t started = PerformanceCounterNow();
    InvalidateAccessRegions();
    for (std::size_t reverse = g_articulatedPoses.size(); reverse > 0U;
         --reverse) {
        ArticulatedPoseEntry& entry = g_articulatedPoses[reverse - 1U];
        if (!RestoreArticulatedPose(entry)) {
            ++entry.restoreFailures;
            ClearArticulatedPose(entry);
        }
    }
    for (std::size_t reverse = g_scriptedPoses.size(); reverse > 0U;
         --reverse) {
        ScriptedPoseEntry& entry = g_scriptedPoses[reverse - 1U];
        if (!RestoreScriptedPose(entry)) {
            ++entry.restoreFailures;
            ResetScriptedPose(entry, true);
        }
    }
    if (!RestoreBodyPose()) {
        ++g_poseRestoreFailures;
        ResetBodyPose();
    }
    for (std::size_t reverse = g_appliedWorldBodies.size(); reverse > 0U;
         --reverse) {
        const std::size_t index = reverse - 1U;
        if (!RestoreWorldBody(g_appliedWorldBodies[index])) {
            ++g_worldRestoreFailures;
            g_worldBodies[index].body = nullptr;
            g_worldBodies[index].history.Reset();
        }
    }
    for (std::size_t reverse = g_appliedInspectionRoots.size(); reverse > 0U;
         --reverse) {
        const std::size_t index = reverse - 1U;
        if (!Restore(g_appliedInspectionRoots[index])) {
            ++g_rootRestoreFailures;
            ++g_inspectionRestoreFailures;
            ResetInspectionRoot(g_inspectionRoots[index]);
        }
    }
    for (std::size_t reverse = g_appliedScriptedRoots.size(); reverse > 0U;
         --reverse) {
        const std::size_t index = reverse - 1U;
        if (!Restore(g_appliedScriptedRoots[index])) {
            ++g_rootRestoreFailures;
            ++g_scriptedRootRestoreFailures;
            ResetScriptedRoot(g_scriptedRoots[index]);
        }
    }
    for (std::size_t reverse = g_appliedHeldAttachments.size(); reverse > 0U;
         --reverse) {
        const std::size_t index = reverse - 1U;
        if (!Restore(g_appliedHeldAttachments[index])) {
            ++g_rootRestoreFailures;
            ++g_heldRestoreFailures;
            Reset(g_heldAttachments[index]);
        }
    }
    if (!Restore(g_appliedBody)) {
        ++g_rootRestoreFailures;
        g_body.history.Reset();
    }
    if (!Restore(g_appliedHands)) {
        ++g_rootRestoreFailures;
        g_hands.history.Reset();
    }
    g_renderActive = false;
    g_viewYawLeadActive = false;
    AccumulatePerformance(started, g_renderRestoreTicks,
                          g_renderRestoreCalls);
}

void Stop() noexcept {
    if (g_executableBase == nullptr) {
        return;
    }
    EndRender();
    std::byte* const target = g_executableBase + kEntitySetMatrixRva;
    if (g_originalLuxAgentOnUpdate != nullptr) {
        std::byte* const agentTarget =
            g_executableBase + kLuxAgentOnUpdateRva;
        static_cast<void>(MH_DisableHook(agentTarget));
        static_cast<void>(MH_RemoveHook(agentTarget));
        g_originalLuxAgentOnUpdate = nullptr;
    }
    if (g_originalLuxPropOnUpdate != nullptr) {
        std::byte* const luxPropOnUpdateTarget =
            g_executableBase + kLuxPropOnUpdateRva;
        static_cast<void>(MH_DisableHook(luxPropOnUpdateTarget));
        static_cast<void>(MH_RemoveHook(luxPropOnUpdateTarget));
        g_originalLuxPropOnUpdate = nullptr;
    }
    if (g_originalEntitySetVarBool != nullptr) {
        std::byte* const setVarBoolTarget =
            g_executableBase + kEntitySetVarBoolRva;
        static_cast<void>(MH_DisableHook(setVarBoolTarget));
        static_cast<void>(MH_RemoveHook(setVarBoolTarget));
        g_originalEntitySetVarBool = nullptr;
    }
    if (g_originalNewtonTransformCallback != nullptr) {
        std::byte* const newtonTransformTarget =
            g_executableBase + kNewtonTransformCallbackRva;
        static_cast<void>(MH_DisableHook(newtonTransformTarget));
        static_cast<void>(MH_RemoveHook(newtonTransformTarget));
        g_originalNewtonTransformCallback = nullptr;
        g_newtonBodyGetUserData = nullptr;
    }
    if (g_originalEntityNameResolver != nullptr) {
        std::byte* const resolverTarget =
            g_executableBase + kEntityNameResolverRva;
        static_cast<void>(MH_DisableHook(resolverTarget));
        static_cast<void>(MH_RemoveHook(resolverTarget));
        g_originalEntityNameResolver = nullptr;
    }
    static_cast<void>(MH_DisableHook(target));
    static_cast<void>(MH_RemoveHook(target));
    g_originalEntitySetMatrix = nullptr;
    g_entityGetMatrix = nullptr;
    g_matrixInverse = nullptr;
    g_matrixMultiply = nullptr;
    g_nodeMatrixChanged = nullptr;
    g_boneSetMatrix = nullptr;
    g_executableBase = nullptr;
    g_interpolationEnabled = false;
    g_poseInterpolationEnabled = false;
    g_worldInterpolationEnabled = false;
    g_articulatedPoseInterpolationEnabled = false;
    g_diagnosticsEnabled = false;
    g_performanceFrequency = {};
    g_bodyBoneProbe = {};
    g_handsBoneProbe = {};
    ResetBodyPose();
    for (ScriptedPoseEntry& entry : g_scriptedPoses) {
        ResetScriptedPose(entry, true);
        entry.registrationLogged = false;
        entry.activationLogged = false;
    }
    for (TrackedRoot& tracked : g_heldAttachments) {
        Reset(tracked);
    }
    for (TrackedNamedRoot& entry : g_scriptedRoots) {
        ResetScriptedRoot(entry);
    }
    for (TrackedNamedRoot& entry : g_inspectionRoots) {
        ResetInspectionRoot(entry);
    }
    g_inspectionIdentities = {};
    g_appliedHeldAttachments = {};
    g_appliedScriptedRoots = {};
    g_appliedInspectionRoots = {};
    while (g_worldBodyProbeLock.test_and_set(std::memory_order_acquire)) {
        SwitchToThread();
    }
    g_worldBodyProbes = {};
    g_worldBodyProbeLock.clear(std::memory_order_release);
    while (g_worldStageLock.test_and_set(std::memory_order_acquire)) {
        SwitchToThread();
    }
    g_stagedWorldBodies = {};
    g_worldStageLock.clear(std::memory_order_release);
    g_worldBodyProbeLockDrops = 0U;
    g_worldBodyProbeInvalid = 0U;
    g_worldBodyProbeOverflows = 0U;
    g_worldStageOverflows = 0U;
    while (g_entityRootProbeLock.test_and_set(std::memory_order_acquire)) {
        SwitchToThread();
    }
    g_entityRootProbes = {};
    g_entityRootProbeLock.clear(std::memory_order_release);
    g_entityRootProbeLockDrops = 0U;
    g_entityRootProbeInvalid = 0U;
    g_entityRootProbeOverflows = 0U;
    for (TrackedWorldBody& tracked : g_worldBodies) {
        tracked.body = nullptr;
        tracked.history.Reset();
    }
    g_appliedWorldBodies = {};
    g_articulatedPoses = {};
    g_loggedWorldInterpolation = false;
    g_loggedArticulatedPoseInterpolation = false;
    g_loggedInspectionInterpolation = false;
    Reset(g_hands);
    Reset(g_body);
}

} // namespace arb::root_probe
