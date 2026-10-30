#include "camera_hooks.h"

#include "camera_interpolation.h"
#include "engine_hooks.h"
#include "game_build.h"
#include "gui_cursor.h"
#include "logger.h"
#include "mouse_hooks.h"
#include "particle_hooks.h"
#include "presentation_clock.h"
#include "root_probe.h"
#include "swap_telemetry.h"

#include <MinHook.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace arb::camera_hooks {
namespace {

using build::Function;

constexpr std::size_t kViewportCameraOffset = 0x18U;
constexpr std::size_t kViewportWorldOffset = 0x20U;
constexpr std::size_t kViewportListenerOffset = 0x2BU;
constexpr std::size_t kCameraPositionOffset = 0x10U;
constexpr std::size_t kCameraPitchOffset = 0x48U;
constexpr std::size_t kCameraYawOffset = 0x4CU;
constexpr std::size_t kCameraRollOffset = 0x50U;
constexpr std::size_t kCameraRotateModeOffset = 0x70U;
constexpr std::size_t kCameraRotationMatrixOffset = 0x1F8U;
constexpr std::size_t kCameraViewDirtyOffset = 0x6F1U;
constexpr std::size_t kCameraRotationDirtyOffset = 0x6F3U;
constexpr std::size_t kCameraWorldDirtyOffset = 0x6F4U;
constexpr std::size_t kCameraFrustumDirtyOffset = 0x6F5U;
constexpr std::size_t kEngineLogicTimerOffset = 0x48U;
constexpr std::size_t kLogicTimerNextTickOffset = 0x00U;
constexpr std::size_t kLogicTimerStepMicrosecondsOffset = 0x08U;
constexpr std::size_t kLogicTimerSpeedMultiplierOffset = 0x30U;
constexpr int kEulerRotateMode = 0;
constexpr int kMatrixRotateMode = 1;
constexpr int kPostUpdateMessage = 5;

using UpdaterRunMessage = void(__fastcall*)(void* updater, int message,
                                             float timeStep);
using ViewportRender = void(__fastcall*)(void* scene, std::byte* viewport,
                                         float frameTime,
                                         std::uint32_t flags);
using SetPosition = void(__fastcall*)(std::byte* camera,
                                      const camera::Vec3* position);
using SetRotationMatrix = void(__fastcall*)(
    std::byte* camera, const root::Matrix* rotationMatrix);
using CurrentMicroseconds = std::uint64_t(__fastcall*)();

UpdaterRunMessage g_originalUpdaterRunMessage = nullptr;
ViewportRender g_originalViewportRender = nullptr;
SetPosition g_setPosition = nullptr;
SetRotationMatrix g_setRotationMatrix = nullptr;
CurrentMicroseconds g_currentMicroseconds = nullptr;

clock::PresentationClock g_clock;
camera::History g_history;
std::uint64_t g_tickSerial = 0;
std::int64_t g_qpcFrequency = 0;
void* g_viewport = nullptr;
void* g_world = nullptr;
std::byte* g_camera = nullptr;
std::atomic<bool> g_loggedActivation{false};
std::atomic<bool> g_loggedClockActivation{false};
std::atomic<bool> g_loggedListenerActivation{false};
std::atomic<bool> g_loggedMatrixModeActivation{false};
std::atomic<bool> g_loggedInvalidLayout{false};
std::atomic<bool> g_loggedRestoreFailure{false};
std::atomic<bool> g_loggedPendingStatus{false};
std::atomic<std::uint32_t> g_listenerFrames{0};

enum class PoseApplyResult : std::size_t {
    applied,
    invalidInput,
    modeMismatch,
    readbackInvalid,
    positionMismatch,
    orientationMismatch,
    count,
};

struct Diagnostics {
    std::array<std::uint64_t, 7> publishes{};
    std::array<std::uint64_t, 5> samples{};
    std::uint64_t frames = 0;
    std::uint64_t eulerFrames = 0;
    std::uint64_t matrixFrames = 0;
    std::uint64_t applied = 0;
    std::uint64_t phaseInvalid = 0;
    std::uint64_t alphaClampedLow = 0;
    std::uint64_t alphaClampedHigh = 0;
    std::uint64_t applyFailures = 0;
    std::uint64_t restoreFailures = 0;
    std::array<std::uint64_t,
               static_cast<std::size_t>(PoseApplyResult::count)>
        applyResults{};
    std::array<std::uint64_t,
               static_cast<std::size_t>(PoseApplyResult::count)>
        restoreResults{};
    std::uint64_t failedApplyRollbackFailures = 0;
    double lastReportSeconds = 0.0;
};

Diagnostics g_diagnostics;

std::wstring MinHookError(const wchar_t* operation,
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

bool ReadQpcSeconds(double& seconds) noexcept {
    LARGE_INTEGER counter{};
    if (g_qpcFrequency <= 0 || !QueryPerformanceCounter(&counter)) {
        return false;
    }
    seconds = static_cast<double>(counter.QuadPart) /
              static_cast<double>(g_qpcFrequency);
    return std::isfinite(seconds);
}

template <typename T>
T ReadAt(const std::byte* const base, const std::size_t offset) noexcept {
    T result{};
    std::memcpy(&result, base + offset, sizeof(result));
    return result;
}

template <typename T>
void WriteAt(std::byte* const base, const std::size_t offset,
             const T& value) noexcept {
    std::memcpy(base + offset, &value, sizeof(value));
}

bool ReadPose(const std::byte* const cameraPointer,
              camera::Pose& pose) noexcept {
    if (cameraPointer == nullptr) {
        return false;
    }
    const int rotateMode = ReadAt<int>(cameraPointer, kCameraRotateModeOffset);
    if (rotateMode == kEulerRotateMode) {
        pose.rotateMode = camera::RotateMode::eulerAngles;
        pose.pitch = ReadAt<float>(cameraPointer, kCameraPitchOffset);
        pose.yaw = ReadAt<float>(cameraPointer, kCameraYawOffset);
        pose.roll = ReadAt<float>(cameraPointer, kCameraRollOffset);
    } else if (rotateMode == kMatrixRotateMode) {
        pose.rotateMode = camera::RotateMode::matrix;
        pose.rotationMatrix = ReadAt<root::Matrix>(
            cameraPointer, kCameraRotationMatrixOffset);
    } else {
        return false;
    }
    pose.position = ReadAt<camera::Vec3>(cameraPointer, kCameraPositionOffset);
    return camera::IsFinite(pose);
}

PoseApplyResult ApplyPose(std::byte* const cameraPointer,
                          const camera::Pose& pose) noexcept {
    if (cameraPointer == nullptr || !camera::IsFinite(pose) ||
        g_setPosition == nullptr || g_setRotationMatrix == nullptr) {
        return PoseApplyResult::invalidInput;
    }
    const int expectedMode =
        pose.rotateMode == camera::RotateMode::matrix ? kMatrixRotateMode
                                                       : kEulerRotateMode;
    if (ReadAt<int>(cameraPointer, kCameraRotateModeOffset) != expectedMode) {
        return PoseApplyResult::modeMismatch;
    }
    g_setPosition(cameraPointer, &pose.position);
    if (pose.rotateMode == camera::RotateMode::matrix) {
        g_setRotationMatrix(cameraPointer, &pose.rotationMatrix);
    } else {
        WriteAt(cameraPointer, kCameraPitchOffset, pose.pitch);
        WriteAt(cameraPointer, kCameraYawOffset, pose.yaw);
        WriteAt(cameraPointer, kCameraRollOffset, pose.roll);
    }
    const std::uint8_t dirty = 1U;
    WriteAt(cameraPointer, kCameraViewDirtyOffset, dirty);
    WriteAt(cameraPointer, kCameraRotationDirtyOffset, dirty);
    WriteAt(cameraPointer, kCameraWorldDirtyOffset, dirty);
    WriteAt(cameraPointer, kCameraFrustumDirtyOffset, dirty);

    camera::Pose applied{};
    if (!ReadPose(cameraPointer, applied)) {
        return PoseApplyResult::readbackInvalid;
    }
    if (std::abs(applied.position.x - pose.position.x) >= 1.0e-4F ||
        std::abs(applied.position.y - pose.position.y) >= 1.0e-4F ||
        std::abs(applied.position.z - pose.position.z) >= 1.0e-4F) {
        return PoseApplyResult::positionMismatch;
    }
    if (camera::OrientationDistance(applied, pose) >= 1.0e-3F) {
        return PoseApplyResult::orientationMismatch;
    }
    return PoseApplyResult::applied;
}

bool ReadSchedulerAlpha(double& alpha, double& rawAlpha) noexcept {
    alpha = 0.0;
    rawAlpha = 0.0;
    std::byte* const engineInstance = engine::Instance();
    if (engineInstance == nullptr || g_currentMicroseconds == nullptr) {
        return false;
    }
    const std::byte* const logicTimer =
        ReadAt<std::byte*>(engineInstance, kEngineLogicTimerOffset);
    if (logicTimer == nullptr) {
        return false;
    }
    const std::uint64_t nextTick =
        ReadAt<std::uint64_t>(logicTimer, kLogicTimerNextTickOffset);
    const double stepMicroseconds = ReadAt<double>(
        logicTimer, kLogicTimerStepMicrosecondsOffset);
    const double speedMultiplier = ReadAt<double>(
        logicTimer, kLogicTimerSpeedMultiplierOffset);
    const std::uint64_t now = g_currentMicroseconds();
    return clock::LogicTimerAlpha(
        static_cast<double>(nextTick) / 1000.0,
        stepMicroseconds / 1000.0, speedMultiplier,
        static_cast<double>(now) / 1000.0, 0.0, alpha, &rawAlpha);
}

void ReportDiagnostics(const double now) {
    if (g_diagnostics.lastReportSeconds == 0.0) {
        g_diagnostics.lastReportSeconds = now;
        return;
    }
    if (now - g_diagnostics.lastReportSeconds < 5.0) {
        return;
    }
    log::Write(
        log::Level::info,
        L"Camera cadence: frames=" + std::to_wstring(g_diagnostics.frames) +
        L", modes(euler/matrix)=" +
            std::to_wstring(g_diagnostics.eulerFrames) + L"/" +
            std::to_wstring(g_diagnostics.matrixFrames) +
        L", applied=" + std::to_wstring(g_diagnostics.applied) +
        L", phase-invalid=" + std::to_wstring(g_diagnostics.phaseInvalid) +
        L", alpha-low/high=" +
            std::to_wstring(g_diagnostics.alphaClampedLow) + L"/" +
            std::to_wstring(g_diagnostics.alphaClampedHigh) +
        L", publishes(appended/unchanged/same-tick/discontinuity)=" +
            std::to_wstring(g_diagnostics.publishes[
                static_cast<std::size_t>(camera::PublishKind::appended)]) + L"/" +
            std::to_wstring(g_diagnostics.publishes[
                static_cast<std::size_t>(camera::PublishKind::unchanged)]) + L"/" +
            std::to_wstring(g_diagnostics.publishes[
                static_cast<std::size_t>(camera::PublishKind::updatedSameTick)]) + L"/" +
            std::to_wstring(g_diagnostics.publishes[
                static_cast<std::size_t>(camera::PublishKind::resetDiscontinuity)]) +
        L", failures(apply/restore)=" +
            std::to_wstring(g_diagnostics.applyFailures) + L"/" +
            std::to_wstring(g_diagnostics.restoreFailures) +
        L", failure-detail(input/mode/readback/position/orientation)=" +
            std::to_wstring(g_diagnostics.applyResults[
                static_cast<std::size_t>(PoseApplyResult::invalidInput)]) +
            L"/" +
            std::to_wstring(g_diagnostics.applyResults[
                static_cast<std::size_t>(PoseApplyResult::modeMismatch)]) +
            L"/" +
            std::to_wstring(g_diagnostics.applyResults[
                static_cast<std::size_t>(PoseApplyResult::readbackInvalid)]) +
            L"/" +
            std::to_wstring(g_diagnostics.applyResults[
                static_cast<std::size_t>(PoseApplyResult::positionMismatch)]) +
            L"/" +
            std::to_wstring(g_diagnostics.applyResults[
                static_cast<std::size_t>(
                    PoseApplyResult::orientationMismatch)]) +
            L", restore=" +
            std::to_wstring(g_diagnostics.restoreResults[
                static_cast<std::size_t>(PoseApplyResult::invalidInput)]) +
            L"/" +
            std::to_wstring(g_diagnostics.restoreResults[
                static_cast<std::size_t>(PoseApplyResult::modeMismatch)]) +
            L"/" +
            std::to_wstring(g_diagnostics.restoreResults[
                static_cast<std::size_t>(PoseApplyResult::readbackInvalid)]) +
            L"/" +
            std::to_wstring(g_diagnostics.restoreResults[
                static_cast<std::size_t>(PoseApplyResult::positionMismatch)]) +
            L"/" +
            std::to_wstring(g_diagnostics.restoreResults[
                static_cast<std::size_t>(
                    PoseApplyResult::orientationMismatch)]) +
            L", failed-apply-rollback=" +
            std::to_wstring(g_diagnostics.failedApplyRollbackFailures) +
            L".");
    g_diagnostics = {};
    g_diagnostics.lastReportSeconds = now;
}

void ResetCameraIdentity() noexcept {
    g_history.Reset();
    root_probe::ResetHistory();
    mouse_hooks::ResetHistory();
    g_viewport = nullptr;
    g_world = nullptr;
    g_camera = nullptr;
}

void __fastcall UpdaterRunMessageHook(void* const updater, const int message,
                                      const float timeStep) noexcept {
    g_originalUpdaterRunMessage(updater, message, timeStep);
    if (message != kPostUpdateMessage || !std::isfinite(timeStep) ||
        timeStep < 0.001F || timeStep > 1.0F) {
        return;
    }

    double now = 0.0;
    if (!ReadQpcSeconds(now)) {
        return;
    }
    if (!g_clock.Configure(static_cast<double>(timeStep))) {
        return;
    }
    ++g_tickSerial;
    const clock::PublishResult published = g_clock.Publish(g_tickSerial, now);
    if (!published.accepted) {
        ResetCameraIdentity();
        return;
    }
    root_probe::Commit(g_tickSerial, g_clock.Generation());
    mouse_hooks::OnTickCompleted(g_tickSerial);
    particle_hooks::OnTickCompleted();
    gui_cursor::OnTickCompleted();
    if (!g_loggedClockActivation.exchange(true,
                                           std::memory_order_acq_rel)) {
        log::Write(log::Level::info,
                   L"Camera interpolation clock active: fixed step=" +
                       std::to_wstring(static_cast<double>(timeStep) * 1000.0) +
                       L" ms, completed-step message=5.");
    }
}

void __fastcall ViewportRenderHook(void* const scene, std::byte* const viewport,
                                   const float frameTime,
                                   const std::uint32_t flags) noexcept {
    if (viewport == nullptr || !g_clock.Configured() || g_tickSerial == 0U) {
        g_originalViewportRender(scene, viewport, frameTime, flags);
        return;
    }
    const std::uint8_t isListener =
        ReadAt<std::uint8_t>(viewport, kViewportListenerOffset);
    if (isListener == 0U) {
        g_originalViewportRender(scene, viewport, frameTime, flags);
        return;
    }
    if (isListener != 1U) {
        ResetCameraIdentity();
        g_originalViewportRender(scene, viewport, frameTime, flags);
        return;
    }
    if (!g_loggedListenerActivation.exchange(true,
                                              std::memory_order_acq_rel)) {
        log::Write(log::Level::info,
                   L"Camera interpolation found the active listener viewport "
                   L"at validated offset +0x2B.");
    }

    std::byte* const cameraPointer =
        ReadAt<std::byte*>(viewport, kViewportCameraOffset);
    void* const world = ReadAt<void*>(viewport, kViewportWorldOffset);
    camera::Pose authoritative{};
    if (cameraPointer == nullptr || world == nullptr ||
        !ReadPose(cameraPointer, authoritative)) {
        ResetCameraIdentity();
        if (!g_loggedInvalidLayout.exchange(true, std::memory_order_acq_rel)) {
            log::Write(log::Level::warning,
                       L"Camera interpolation skipped a viewport whose "
                       L"validated Rebirth camera layout was unavailable.");
        }
        g_originalViewportRender(scene, viewport, frameTime, flags);
        return;
    }

    if (g_viewport != viewport || g_world != world ||
        g_camera != cameraPointer) {
        ResetCameraIdentity();
        g_viewport = viewport;
        g_world = world;
        g_camera = cameraPointer;
    }

    const camera::PublishResult posePublish = g_history.Publish(
        g_tickSerial, g_clock.Generation(), authoritative);
    ++g_diagnostics.frames;
    if (authoritative.rotateMode == camera::RotateMode::matrix) {
        ++g_diagnostics.matrixFrames;
        if (!g_loggedMatrixModeActivation.exchange(true,
                                                    std::memory_order_acq_rel)) {
            log::Write(log::Level::info,
                       L"Camera interpolation entered validated matrix mode; "
                       L"scripted CameraSocket motion is now eligible for "
                       L"render interpolation.");
        }
    } else {
        ++g_diagnostics.eulerFrames;
    }
    ++g_diagnostics.publishes[static_cast<std::size_t>(posePublish.kind)];
    if (posePublish.kind == camera::PublishKind::invalid) {
        g_originalViewportRender(scene, viewport, frameTime, flags);
        return;
    }

    // Wait for the frame's slot before sampling time, phase, and mouse. If
    // the wait came after rendering, frames that follow a fixed tick would
    // present sooner after sampling than the others, which shows as uneven
    // motion at the tick rate.
    telemetry::PaceBeforeRender();
    gui_cursor::OnFrame();

    double now = 0.0;
    if (!ReadQpcSeconds(now)) {
        g_originalViewportRender(scene, viewport, frameTime, flags);
        return;
    }
    double alpha = 0.0;
    double rawAlpha = 0.0;
    const bool phaseValid = ReadSchedulerAlpha(alpha, rawAlpha);
    if (!phaseValid) {
        ++g_diagnostics.phaseInvalid;
    } else if (rawAlpha < 0.0) {
        ++g_diagnostics.alphaClampedLow;
    } else if (rawAlpha > 1.0) {
        ++g_diagnostics.alphaClampedHigh;
    }
    const clock::Sample clockSample =
        g_clock.Evaluate(now, alpha, phaseValid);
    const camera::SampleResult sampled = g_history.Evaluate(clockSample);
    ++g_diagnostics.samples[static_cast<std::size_t>(sampled.kind)];
    ReportDiagnostics(now);
    if (sampled.kind != camera::SampleKind::applied) {
        const std::uint32_t listenerFrames =
            g_listenerFrames.fetch_add(1U, std::memory_order_relaxed) + 1U;
        if (listenerFrames >= 240U &&
            !g_loggedPendingStatus.exchange(true, std::memory_order_acq_rel)) {
            log::Write(log::Level::warning,
                       L"Camera interpolation has not produced a sample after "
                       L"240 listener frames: pose-publish=" +
                           std::wstring(camera::PublishKindName(posePublish.kind)) +
                           L", clock-sample=" +
                           std::wstring(clock::SampleKindName(clockSample.kind)) +
                           L", camera-sample=" +
                           std::wstring(camera::SampleKindName(sampled.kind)) +
                           L", alpha=" + std::to_wstring(alpha) +
                           L", raw-alpha=" + std::to_wstring(rawAlpha) + L".");
        }
        g_originalViewportRender(scene, viewport, frameTime, flags);
        return;
    }
    // Mouse look is shown ahead of the fixed tick that will apply it: the
    // interpolated camera gains the rotation the game is about to add. The
    // authoritative camera and the input the game receives are unchanged.
    camera::Pose presented = sampled.pose;
    mouse_hooks::Lead lead{};
    camera::Pose olderEndpoint{};
    camera::Pose newerEndpoint{};
    const bool hasEndpoints =
        g_history.Endpoints(clockSample, olderEndpoint, newerEndpoint);
    // Mouse motion is gathered every frame in every camera mode; a lead is
    // only ever returned for the mouse-driven (Euler) camera.
    if (mouse_hooks::Enabled() && hasEndpoints) {
        lead = mouse_hooks::Sample(clockSample, olderEndpoint, newerEndpoint);
        if (lead.active &&
            presented.rotateMode == camera::RotateMode::eulerAngles) {
            constexpr float maximumPresentedPitch = 1.55F;
            presented.yaw += lead.yaw;
            presented.pitch = std::clamp(presented.pitch + lead.pitch,
                                         -maximumPresentedPitch,
                                         maximumPresentedPitch);
        }
    }
    // An animation-driven camera is placed on the interpolated skeleton so
    // it stays locked to the body it is attached to between ticks.
    if (hasEndpoints && presented.rotateMode == camera::RotateMode::matrix &&
        olderEndpoint.rotateMode == camera::RotateMode::matrix &&
        newerEndpoint.rotateMode == camera::RotateMode::matrix) {
        const attach::CameraEndpoint olderCamera{
            olderEndpoint.rotationMatrix, olderEndpoint.position.x,
            olderEndpoint.position.y, olderEndpoint.position.z};
        const attach::CameraEndpoint newerCamera{
            newerEndpoint.rotationMatrix, newerEndpoint.position.x,
            newerEndpoint.position.y, newerEndpoint.position.z};
        attach::CameraEndpoint attached{
            presented.rotationMatrix, presented.position.x,
            presented.position.y, presented.position.z};
        if (root_probe::SolveAttachedCamera(clockSample, olderCamera,
                                            newerCamera, attached)) {
            camera::Pose candidate = presented;
            candidate.rotationMatrix = attached.rotation;
            candidate.position = {attached.x, attached.y, attached.z};
            if (camera::IsFinite(candidate)) {
                presented = candidate;
            }
        }
    }
    const PoseApplyResult applyResult = ApplyPose(cameraPointer, presented);
    if (applyResult != PoseApplyResult::applied) {
        ++g_diagnostics.applyFailures;
        ++g_diagnostics.applyResults[static_cast<std::size_t>(applyResult)];
        if (ApplyPose(cameraPointer, authoritative) !=
            PoseApplyResult::applied) {
            ++g_diagnostics.failedApplyRollbackFailures;
        }
        g_originalViewportRender(scene, viewport, frameTime, flags);
        return;
    }
    ++g_diagnostics.applied;

    if (!g_loggedActivation.exchange(true, std::memory_order_acq_rel)) {
        log::Write(log::Level::info,
                   L"Camera interpolation active on the validated Rebirth "
                   L"viewport/camera layout.");
    }
    // The first-person body and held items turn with the camera, so they get
    // the same yaw lead about the camera's position.
    root_probe::SetViewYawLead(lead.active ? lead.yaw : 0.0F,
                               presented.position.x, presented.position.y,
                               presented.position.z);
    static_cast<void>(root_probe::BeginRender(clockSample));
    particle_hooks::BeginRender(clockSample);
    g_originalViewportRender(scene, viewport, frameTime, flags);
    particle_hooks::EndRender();
    root_probe::EndRender();
    const PoseApplyResult restoreResult =
        ApplyPose(cameraPointer, authoritative);
    if (restoreResult != PoseApplyResult::applied) {
        ++g_diagnostics.restoreFailures;
        ++g_diagnostics.restoreResults[
            static_cast<std::size_t>(restoreResult)];
        if (!g_loggedRestoreFailure.exchange(true,
                                             std::memory_order_acq_rel)) {
        log::Write(log::Level::error,
                   L"Camera interpolation could not restore authoritative "
                   L"camera state exactly; interpolation history was reset.");
        }
        ResetCameraIdentity();
    }
}

} // namespace

bool Start(const HMODULE executable, std::wstring& error) noexcept {
    if (executable == nullptr) {
        error = L"Host module is null while installing camera interpolation.";
        return false;
    }
    auto* const base = reinterpret_cast<std::byte*>(executable);
    auto* const updaterTarget = base + build::Rva(Function::updaterRunMessage);
    auto* const viewportTarget = base + build::Rva(Function::viewportRender);
    auto* const setPositionTarget =
        base + build::Rva(Function::cameraSetPosition);
    auto* const setRotationMatrixTarget =
        base + build::Rva(Function::cameraSetRotationMatrix);
    auto* const currentMicrosecondsTarget =
        base + build::Rva(Function::currentMicroseconds);
    if (!build::EntryMatches(executable, Function::updaterRunMessage) ||
        !build::EntryMatches(executable, Function::viewportRender) ||
        !build::EntryMatches(executable, Function::cameraSetRotationMatrix) ||
        !build::EntryMatches(executable, Function::cameraSetPosition) ||
        !build::EntryMatches(executable, Function::currentMicroseconds)) {
        error = L"Updater or viewport-render entry bytes do not match the "
                L"supported build, or the scheduler clock is unavailable.";
        return false;
    }

    LARGE_INTEGER frequency{};
    if (!QueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0) {
        error = L"QueryPerformanceFrequency failed.";
        return false;
    }
    g_qpcFrequency = frequency.QuadPart;
    g_setPosition = reinterpret_cast<SetPosition>(setPositionTarget);
    g_setRotationMatrix = reinterpret_cast<SetRotationMatrix>(
        setRotationMatrixTarget);
    g_currentMicroseconds = reinterpret_cast<CurrentMicroseconds>(
        currentMicrosecondsTarget);

    MH_STATUS status = MH_Initialize();
    if (status != MH_OK && status != MH_ERROR_ALREADY_INITIALIZED) {
        error = MinHookError(L"MH_Initialize", status);
        return false;
    }
    status = MH_CreateHook(updaterTarget,
                           reinterpret_cast<void*>(&UpdaterRunMessageHook),
                           reinterpret_cast<void**>(&g_originalUpdaterRunMessage));
    if (status != MH_OK) {
        error = MinHookError(L"MH_CreateHook(cUpdater::RunMessage)", status);
        return false;
    }
    status = MH_CreateHook(viewportTarget,
                           reinterpret_cast<void*>(&ViewportRenderHook),
                           reinterpret_cast<void**>(&g_originalViewportRender));
    if (status != MH_OK) {
        MH_RemoveHook(updaterTarget);
        g_originalUpdaterRunMessage = nullptr;
        error = MinHookError(L"MH_CreateHook(cScene::RenderViewport)", status);
        return false;
    }
    status = MH_EnableHook(updaterTarget);
    if (status == MH_OK) {
        status = MH_EnableHook(viewportTarget);
    }
    if (status != MH_OK) {
        MH_DisableHook(updaterTarget);
        MH_RemoveHook(viewportTarget);
        MH_RemoveHook(updaterTarget);
        g_originalViewportRender = nullptr;
        g_originalUpdaterRunMessage = nullptr;
        error = MinHookError(L"MH_EnableHook(camera interpolation)", status);
        return false;
    }
    return true;
}

void Stop() noexcept {
    const HMODULE executable = GetModuleHandleW(nullptr);
    if (executable == nullptr) {
        return;
    }
    auto* const base = reinterpret_cast<std::byte*>(executable);
    auto* const viewportTarget = base + build::Rva(Function::viewportRender);
    auto* const updaterTarget = base + build::Rva(Function::updaterRunMessage);
    static_cast<void>(MH_DisableHook(viewportTarget));
    static_cast<void>(MH_DisableHook(updaterTarget));
    static_cast<void>(MH_RemoveHook(viewportTarget));
    static_cast<void>(MH_RemoveHook(updaterTarget));
    g_originalViewportRender = nullptr;
    g_originalUpdaterRunMessage = nullptr;
    g_setRotationMatrix = nullptr;
    ResetCameraIdentity();
}

} // namespace arb::camera_hooks
