#include "mouse_hooks.h"

#include "game_build.h"
#include "iat_hook.h"
#include "logger.h"
#include "mouse_preview.h"

#include <MinHook.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace arb::mouse_hooks {
namespace {

// Function::lookAmount is the cLuxInputHandler look-amount routine: it reads
// the mouse's relative counts, scales them by sensitivity * system mouse
// speed / screen height, optionally smooths them, and applies vertical
// inversion.
using build::Function;
constexpr std::size_t kGameInputStateOffset = 0xD8U;
constexpr std::size_t kInputStateSmoothingBypassOffset = 0x40EU;
constexpr std::size_t kHandlerSmoothMouseOffset = 0x168U;
constexpr std::size_t kHandlerInvertMouseOffset = 0x169U;
constexpr std::size_t kHandlerMaximumSmoothEntriesOffset = 0x17CU;
constexpr std::size_t kHandlerPreviousSmoothMultiplierOffset = 0x180U;
constexpr std::size_t kHandlerSmoothListHeadOffset = 0x188U;
constexpr std::size_t kHandlerSmoothListSizeOffset = 0x190U;
constexpr std::size_t kListNodeValueOffset = 0x10U;

constexpr float kMaximumScale = 0.1F;
constexpr float kMinimumScale = 1.0e-7F;
// A lead beyond this is not a plausible single-tick mouse rotation.
constexpr float kMaximumLeadRadians = 1.5F;
constexpr std::uint32_t kMaximumRawPacketsPerFrame = 512U;
constexpr std::uint32_t kRequiredExplainedTicks = 4U;
constexpr double kLeadFadeSeconds = 0.05;

using GetRelativeMouseState = std::uint32_t(__cdecl*)(int* x, int* y);
using PumpEvents = void(__cdecl*)();
using GetRelativeMouseMode = int(__cdecl*)();
using LookAmount = float*(__fastcall*)(std::byte* handler, float* amount);

template <typename T>
T ReadAt(const std::byte* const base, const std::size_t offset) noexcept {
    T result{};
    std::memcpy(&result, base + offset, sizeof(result));
    return result;
}

struct TickAccumulator {
    int drainX = 0;
    int drainY = 0;
    std::uint32_t drains = 0;
    std::uint32_t lookCalls = 0;
    mouse::Vec2 amount{};
    mouse::SmootherState smoother{};
    bool smootherValid = false;
    bool invert = false;
};

struct TickRecord {
    std::uint64_t serial = 0;
    mouse::Vec2 amount{};
    mouse::Carry carry{};
    bool invert = false;
    bool valid = false;
};

struct Diagnostics {
    std::uint64_t ticks = 0;
    std::uint64_t drains = 0;
    std::uint64_t lookCalls = 0;
    std::uint64_t invalidTicks = 0;
    std::uint64_t yawVerified = 0;
    std::uint64_t yawUnverified = 0;
    std::uint64_t pitchVerified = 0;
    std::uint64_t pitchUnverified = 0;
    std::uint64_t frames = 0;
    std::uint64_t leadFrames = 0;
    std::uint64_t staleFrames = 0;
    std::uint64_t rejectedLeads = 0;
    std::uint64_t rawPackets = 0;
    std::uint32_t maximumRawPacketsPerFrame = 0;
    std::uint32_t maximumGatheredPerFrame = 0;
    std::int64_t countsGathered = 0;
    std::int64_t countsDelivered = 0;
    float maximumLeadYaw = 0.0F;
    float maximumLeadPitch = 0.0F;
    int maximumPending = 0;
    bool smoothing = false;
    int smoothingEntries = 0;
    float smoothingMultiplier = 0.0F;
    // Read-only study of animation-driven (matrix) camera ticks: is the
    // camera's turn there still a fixed multiple of the mouse amount?
    std::uint64_t withheldWarmupTicks = 0;
    std::uint64_t matrixTicks = 0;
    std::uint64_t matrixRowExplained = 0;
    std::uint64_t matrixColumnExplained = 0;
};

GetRelativeMouseState g_realGetRelativeMouseState = nullptr;
PumpEvents g_pumpEvents = nullptr;
GetRelativeMouseMode g_getRelativeMouseMode = nullptr;
LookAmount g_originalLookAmount = nullptr;
void** g_relativeStateSlot = nullptr;
std::byte* g_executableBase = nullptr;
bool g_enabled = false;
bool g_diagnosticsEnabled = false;
DWORD g_gameThread = 0;

// Motion gathered at render time that the game has not read yet.
int g_pendingX = 0;
int g_pendingY = 0;
TickAccumulator g_tick{};
TickRecord g_record{};
mouse::GainEstimator g_estimator;
mouse::Verification g_verified{};
mouse::Verification g_trusted{};
mouse::Vec2 g_leadWeight{};
double g_lastWeightTime = 0.0;
std::int64_t g_frequency = 0;
std::uint32_t g_yawStreak = 0;
std::uint32_t g_pitchStreak = 0;
std::uint64_t g_observedSerial = 0;
float g_scale = 0.0F;
bool g_hasScale = false;
Diagnostics g_diagnostics{};
mouse::GainEstimator g_matrixRowEstimator;
mouse::GainEstimator g_matrixColumnEstimator;
std::uint64_t g_matrixObservedSerial = 0;
ULONGLONG g_lastReportMilliseconds = 0;

bool OnGameThread() noexcept {
    return g_gameThread != 0 && GetCurrentThreadId() == g_gameThread;
}

std::uint32_t __cdecl GetRelativeMouseStateHook(int* const x,
                                                int* const y) noexcept {
    int realX = 0;
    int realY = 0;
    const GetRelativeMouseState real = g_realGetRelativeMouseState;
    if (real == nullptr) {
        return 0U;
    }
    if (!g_enabled || !OnGameThread()) {
        return real(x, y);
    }
    // Hand the game everything since its previous read: what presentation
    // already gathered plus whatever SDL still holds. Each count is delivered
    // exactly once.
    const std::uint32_t buttons = real(&realX, &realY);
    const int totalX = g_pendingX + realX;
    const int totalY = g_pendingY + realY;
    g_pendingX = 0;
    g_pendingY = 0;
    if (x != nullptr) {
        *x = totalX;
    }
    if (y != nullptr) {
        *y = totalY;
    }
    if (x != nullptr && y != nullptr) {
        g_tick.drainX += totalX;
        g_tick.drainY += totalY;
        ++g_tick.drains;
        g_diagnostics.countsDelivered += std::abs(totalX) + std::abs(totalY);
    }
    return buttons;
}

bool SmoothingBypassed() noexcept {
    const std::byte* const game =
        ReadAt<std::byte*>(g_executableBase, build::GameGlobalRva());
    if (game == nullptr) {
        return false;
    }
    const std::byte* const inputState =
        ReadAt<std::byte*>(game, kGameInputStateOffset);
    return inputState != nullptr &&
           ReadAt<std::uint8_t>(inputState,
                                kInputStateSmoothingBypassOffset) != 0U;
}

bool CaptureSmoother(const std::byte* const handler,
                     mouse::SmootherState& state) noexcept {
    state = {};
    const bool smooth =
        ReadAt<std::uint8_t>(handler, kHandlerSmoothMouseOffset) != 0U;
    if (!smooth || SmoothingBypassed()) {
        return true;
    }
    state.active = true;
    state.maximumEntries =
        ReadAt<int>(handler, kHandlerMaximumSmoothEntriesOffset);
    state.previousMultiplier =
        ReadAt<float>(handler, kHandlerPreviousSmoothMultiplierOffset);
    const std::byte* const head =
        ReadAt<std::byte*>(handler, kHandlerSmoothListHeadOffset);
    const auto size =
        ReadAt<std::size_t>(handler, kHandlerSmoothListSizeOffset);
    if (head == nullptr || size > mouse::kMaximumSmoothingEntries) {
        return false;
    }
    const std::byte* node = ReadAt<std::byte*>(head, 0U);
    for (std::size_t index = 0; index < size; ++index) {
        if (node == nullptr || node == head) {
            return false;
        }
        state.entries[index] = {
            ReadAt<float>(node, kListNodeValueOffset),
            ReadAt<float>(node, kListNodeValueOffset + sizeof(float))};
        node = ReadAt<std::byte*>(node, 0U);
    }
    state.count = size;
    return node == head;
}

float* __fastcall LookAmountHook(std::byte* const handler,
                                 float* const amount) noexcept {
    float* const result = g_originalLookAmount(handler, amount);
    if (g_enabled && OnGameThread() && handler != nullptr &&
        amount != nullptr) {
        ++g_tick.lookCalls;
        g_tick.amount = {amount[0], amount[1]};
        g_tick.invert =
            ReadAt<std::uint8_t>(handler, kHandlerInvertMouseOffset) != 0U;
        g_tick.smootherValid = CaptureSmoother(handler, g_tick.smoother);
    }
    return result;
}

// The newest scaled sample divided by the counts that produced it is the
// game's own counts-to-look factor, with no layout assumptions of ours.
void LearnScale(const TickAccumulator& tick) noexcept {
    if (tick.drains != 1U || tick.lookCalls != 1U || !tick.smootherValid) {
        return;
    }
    mouse::Vec2 raw{};
    if (tick.smoother.active) {
        if (tick.smoother.count == 0U) {
            return;
        }
        raw = tick.smoother.entries[0];
    } else {
        raw = {tick.amount.x, tick.invert ? -tick.amount.y : tick.amount.y};
    }
    const bool useX = std::abs(tick.drainX) >= std::abs(tick.drainY);
    const int counts = useX ? tick.drainX : tick.drainY;
    if (std::abs(counts) < 4) {
        return;
    }
    const float scale = (useX ? raw.x : raw.y) / static_cast<float>(counts);
    if (std::isfinite(scale) && scale >= kMinimumScale &&
        scale <= kMaximumScale) {
        g_scale = scale;
        g_hasScale = true;
    }
}

struct YawPitch {
    float yaw = 0.0F;
    float pitch = 0.0F;
};

// Heading of one axis of a rotation matrix, read either as a row or as a
// column, since which one is the camera's forward is not established.
YawPitch AxisAngles(const root::Matrix& matrix, const bool column) noexcept {
    const float x = column ? matrix.values[2] : matrix.values[8];
    const float y = column ? matrix.values[6] : matrix.values[9];
    const float z = matrix.values[10];
    return {std::atan2(x, z), std::asin(std::clamp(y, -1.0F, 1.0F))};
}

void ObserveMatrixTick(const root::Matrix& older,
                       const root::Matrix& newer) noexcept {
    ++g_diagnostics.matrixTicks;
    for (const bool column : {false, true}) {
        const YawPitch before = AxisAngles(older, column);
        const YawPitch after = AxisAngles(newer, column);
        mouse::GainEstimator& estimator =
            column ? g_matrixColumnEstimator : g_matrixRowEstimator;
        const mouse::Verification verified = estimator.Observe(
            g_record.amount, mouse::WrapAngle(after.yaw - before.yaw),
            mouse::WrapAngle(after.pitch - before.pitch));
        if (verified.yaw && verified.pitch) {
            ++(column ? g_diagnostics.matrixColumnExplained
                      : g_diagnostics.matrixRowExplained);
        }
    }
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
    const Diagnostics& d = g_diagnostics;
    log::Write(
        log::Level::info,
        L"Mouse preview: ticks/reads/look-calls/unusable-ticks=" +
            std::to_wstring(d.ticks) + L"/" + std::to_wstring(d.drains) +
            L"/" + std::to_wstring(d.lookCalls) + L"/" +
            std::to_wstring(d.invalidTicks) +
            L", yaw explained/unexplained=" + std::to_wstring(d.yawVerified) +
            L"/" + std::to_wstring(d.yawUnverified) +
            L", pitch explained/unexplained=" +
            std::to_wstring(d.pitchVerified) + L"/" +
            std::to_wstring(d.pitchUnverified) + L", frames/lead/stale/" +
            L"rejected=" + std::to_wstring(d.frames) + L"/" +
            std::to_wstring(d.leadFrames) + L"/" +
            std::to_wstring(d.staleFrames) + L"/" +
            std::to_wstring(d.rejectedLeads) +
            L", counts gathered-at-render/delivered=" +
            std::to_wstring(d.countsGathered) + L"/" +
            std::to_wstring(d.countsDelivered) +
            L", raw packets total/max-per-frame=" +
            std::to_wstring(d.rawPackets) + L"/" +
            std::to_wstring(d.maximumRawPacketsPerFrame) +
            L", max counts gathered in one frame=" +
            std::to_wstring(d.maximumGatheredPerFrame) + L", max pending=" +
            std::to_wstring(d.maximumPending) + L", max lead yaw/pitch=" +
            std::to_wstring(d.maximumLeadYaw) + L"/" +
            std::to_wstring(d.maximumLeadPitch) + L" rad, gain=" +
            (g_estimator.HasGain() ? std::to_wstring(g_estimator.Gain())
                                   : std::wstring(L"unknown")) +
            L", scale=" +
            (g_hasScale ? std::to_wstring(g_scale * 1.0e6F) + L"e-6"
                        : std::wstring(L"unknown")) +
            L", ticks withheld while re-earning trust=" +
            std::to_wstring(d.withheldWarmupTicks) +
            L", matrix-camera ticks/explained by row/column forward=" +
            std::to_wstring(d.matrixTicks) + L"/" +
            std::to_wstring(d.matrixRowExplained) + L"/" +
            std::to_wstring(d.matrixColumnExplained) + L", matrix gain row/" +
            L"column=" +
            (g_matrixRowEstimator.HasGain()
                 ? std::to_wstring(g_matrixRowEstimator.Gain())
                 : std::wstring(L"unknown")) +
            L"/" +
            (g_matrixColumnEstimator.HasGain()
                 ? std::to_wstring(g_matrixColumnEstimator.Gain())
                 : std::wstring(L"unknown")) +
            L", smoothing=" +
            (d.smoothing ? L"on(" + std::to_wstring(d.smoothingEntries) +
                               L"," + std::to_wstring(d.smoothingMultiplier) +
                               L")"
                         : std::wstring(L"off")) +
            L".");
    g_diagnostics = {};
    g_lastReportMilliseconds = now;
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
    return std::wstring(operation) + L" failed: " + converted + L".";
}

} // namespace

bool Start(const HMODULE executable, const bool diagnosticsEnabled,
           std::wstring& error) noexcept {
    if (executable == nullptr) {
        error = L"Host module is null while installing the mouse preview.";
        return false;
    }
    auto* const base = reinterpret_cast<std::byte*>(executable);
    auto* const lookTarget = base + build::Rva(Function::lookAmount);
    LARGE_INTEGER frequency{};
    g_frequency =
        QueryPerformanceFrequency(&frequency) ? frequency.QuadPart : 0;
    if (!build::EntryMatches(executable, Function::lookAmount)) {
        error = L"Look-amount entry bytes do not match the supported build.";
        return false;
    }
    const iat::LocateResult relativeState =
        iat::Locate(executable, "SDL2.dll", "SDL_GetRelativeMouseState");
    const iat::LocateResult pump =
        iat::Locate(executable, "SDL2.dll", "SDL_PumpEvents");
    const iat::LocateResult relativeMode =
        iat::Locate(executable, "SDL2.dll", "SDL_GetRelativeMouseMode");
    if (relativeState.slot == nullptr || relativeState.current == nullptr ||
        pump.current == nullptr || relativeMode.current == nullptr) {
        error = L"Could not locate the SDL relative-mouse imports.";
        return false;
    }

    MH_STATUS status = MH_Initialize();
    if (status != MH_OK && status != MH_ERROR_ALREADY_INITIALIZED) {
        error = MinHookError(L"MH_Initialize", status);
        return false;
    }
    status = MH_CreateHook(lookTarget,
                           reinterpret_cast<void*>(&LookAmountHook),
                           reinterpret_cast<void**>(&g_originalLookAmount));
    if (status != MH_OK) {
        error = MinHookError(L"MH_CreateHook(look amount)", status);
        return false;
    }

    g_executableBase = base;
    g_diagnosticsEnabled = diagnosticsEnabled;
    g_pumpEvents = reinterpret_cast<PumpEvents>(pump.current);
    g_getRelativeMouseMode =
        reinterpret_cast<GetRelativeMouseMode>(relativeMode.current);
    g_realGetRelativeMouseState =
        reinterpret_cast<GetRelativeMouseState>(relativeState.current);
    if (!iat::Replace(relativeState.slot, relativeState.current,
                      reinterpret_cast<void*>(&GetRelativeMouseStateHook),
                      error)) {
        static_cast<void>(MH_RemoveHook(lookTarget));
        g_originalLookAmount = nullptr;
        g_realGetRelativeMouseState = nullptr;
        return false;
    }
    error.clear();
    g_relativeStateSlot = relativeState.slot;
    status = MH_EnableHook(lookTarget);
    if (status != MH_OK) {
        std::wstring ignored;
        static_cast<void>(iat::Replace(
            g_relativeStateSlot,
            reinterpret_cast<void*>(&GetRelativeMouseStateHook),
            reinterpret_cast<void*>(g_realGetRelativeMouseState), ignored));
        static_cast<void>(MH_RemoveHook(lookTarget));
        g_relativeStateSlot = nullptr;
        g_originalLookAmount = nullptr;
        g_realGetRelativeMouseState = nullptr;
        error = MinHookError(L"MH_EnableHook(look amount)", status);
        return false;
    }
    g_enabled = true;
    return true;
}

void Stop() noexcept {
    g_enabled = false;
    if (g_relativeStateSlot != nullptr &&
        g_realGetRelativeMouseState != nullptr) {
        std::wstring ignored;
        static_cast<void>(iat::Replace(
            g_relativeStateSlot,
            reinterpret_cast<void*>(&GetRelativeMouseStateHook),
            reinterpret_cast<void*>(g_realGetRelativeMouseState), ignored));
    }
    g_relativeStateSlot = nullptr;
    if (g_executableBase != nullptr && g_originalLookAmount != nullptr) {
        std::byte* const lookTarget =
            g_executableBase + build::Rva(Function::lookAmount);
        static_cast<void>(MH_DisableHook(lookTarget));
        static_cast<void>(MH_RemoveHook(lookTarget));
    }
    g_originalLookAmount = nullptr;
    g_executableBase = nullptr;
    g_pendingX = 0;
    g_pendingY = 0;
    g_tick = {};
    g_record = {};
    g_estimator.Reset();
    g_hasScale = false;
}

bool Enabled() noexcept {
    return g_enabled;
}

void OnTickCompleted(const std::uint64_t tickSerial) noexcept {
    if (!g_enabled) {
        return;
    }
    if (g_gameThread == 0) {
        g_gameThread = GetCurrentThreadId();
    }
    LearnScale(g_tick);
    ++g_diagnostics.ticks;
    g_diagnostics.drains += g_tick.drains;
    g_diagnostics.lookCalls += g_tick.lookCalls;

    TickRecord record{};
    record.serial = tickSerial;
    // Exactly one look evaluation with a modellable smoother is the only
    // tick shape the prediction is derived for.
    if (g_tick.lookCalls == 1U && g_tick.smootherValid &&
        mouse::PrepareCarry(g_tick.smoother, record.carry)) {
        record.amount = g_tick.amount;
        record.invert = g_tick.invert;
        record.valid = std::isfinite(record.amount.x) &&
                       std::isfinite(record.amount.y);
        g_diagnostics.smoothing = g_tick.smoother.active;
        g_diagnostics.smoothingEntries = g_tick.smoother.maximumEntries;
        g_diagnostics.smoothingMultiplier =
            g_tick.smoother.previousMultiplier;
    }
    if (!record.valid) {
        ++g_diagnostics.invalidTicks;
    }
    g_record = record;
    g_tick = {};
}

void ResetHistory() noexcept {
    g_record = {};
    g_verified = {};
    g_trusted = {};
    g_leadWeight = {};
    g_yawStreak = 0;
    g_pitchStreak = 0;
    g_observedSerial = 0;
}

Lead Sample(const clock::Sample& sample, const camera::Pose& older,
            const camera::Pose& newer) noexcept {
    if (!g_enabled || !OnGameThread() || g_pumpEvents == nullptr ||
        g_getRelativeMouseMode == nullptr ||
        g_realGetRelativeMouseState == nullptr) {
        return {};
    }
    ++g_diagnostics.frames;
    MaybeReport();
    if (g_getRelativeMouseMode() != 1) {
        // A visible cursor is not look input; leave SDL's state untouched.
        return {};
    }

    // Move motion that arrived since the last frame from the OS queue into
    // our pending total. The game's next read receives it unchanged.
    //
    // SDL's own pump stops after the first message stamped with the current
    // GetTickCount value, so a single call per frame leaves most fresh
    // raw-input packets queued until that coarse clock advances and then
    // releases them in one burst. Dispatching the raw-input packets to
    // SDL's window procedure directly drains them all, every frame.
    MSG message{};
    std::uint32_t dispatched = 0;
    while (dispatched < kMaximumRawPacketsPerFrame &&
           PeekMessageW(&message, nullptr, WM_INPUT, WM_INPUT, PM_REMOVE)) {
        DispatchMessageW(&message);
        ++dispatched;
    }
    g_diagnostics.rawPackets += dispatched;
    g_diagnostics.maximumRawPacketsPerFrame =
        std::max(g_diagnostics.maximumRawPacketsPerFrame, dispatched);
    g_pumpEvents();
    int gatheredX = 0;
    int gatheredY = 0;
    static_cast<void>(g_realGetRelativeMouseState(&gatheredX, &gatheredY));
    g_pendingX += gatheredX;
    g_pendingY += gatheredY;
    g_diagnostics.countsGathered += std::abs(gatheredX) + std::abs(gatheredY);
    g_diagnostics.maximumGatheredPerFrame = std::max(
        g_diagnostics.maximumGatheredPerFrame,
        static_cast<std::uint32_t>(std::abs(gatheredX) + std::abs(gatheredY)));
    g_diagnostics.maximumPending =
        std::max({g_diagnostics.maximumPending, std::abs(g_pendingX),
                  std::abs(g_pendingY)});

    if (g_diagnosticsEnabled && g_record.valid &&
        sample.newerSerial == g_record.serial &&
        older.rotateMode == camera::RotateMode::matrix &&
        newer.rotateMode == camera::RotateMode::matrix &&
        g_matrixObservedSerial != g_record.serial) {
        g_matrixObservedSerial = g_record.serial;
        ObserveMatrixTick(older.rotationMatrix, newer.rotationMatrix);
    }
    if (!g_record.valid || sample.newerSerial != g_record.serial ||
        older.rotateMode != camera::RotateMode::eulerAngles ||
        newer.rotateMode != camera::RotateMode::eulerAngles) {
        ++g_diagnostics.staleFrames;
        g_leadWeight = {};
        return {};
    }
    if (g_observedSerial != g_record.serial) {
        g_observedSerial = g_record.serial;
        g_verified = g_estimator.Observe(
            g_record.amount, mouse::WrapAngle(newer.yaw - older.yaw),
            mouse::WrapAngle(newer.pitch - older.pitch));
        ++(g_verified.yaw ? g_diagnostics.yawVerified
                          : g_diagnostics.yawUnverified);
        ++(g_verified.pitch ? g_diagnostics.pitchVerified
                            : g_diagnostics.pitchUnverified);
        // An axis earns the lead only after several consecutive ticks the
        // mouse fully explains. Where the game adds its own look dynamics
        // (look acceleration, damping, scripted pulls) single ticks can
        // match by chance, and switching the lead on and off then shows
        // as jumps.
        g_yawStreak = g_verified.yaw ? g_yawStreak + 1U : 0U;
        g_pitchStreak = g_verified.pitch ? g_pitchStreak + 1U : 0U;
        g_trusted = {g_yawStreak >= kRequiredExplainedTicks,
                     g_pitchStreak >= kRequiredExplainedTicks};
        if ((g_verified.yaw && !g_trusted.yaw) ||
            (g_verified.pitch && !g_trusted.pitch)) {
            ++g_diagnostics.withheldWarmupTicks;
        }
    }
    if (!g_hasScale || !g_estimator.HasGain()) {
        g_leadWeight = {};
        return {};
    }
    // Each axis's lead fades in and out over a short time instead of
    // switching, so a change of trust is never a visible step.
    LARGE_INTEGER counter{};
    const double now = QueryPerformanceCounter(&counter) && g_frequency > 0
        ? static_cast<double>(counter.QuadPart) /
              static_cast<double>(g_frequency)
        : 0.0;
    const double elapsed = g_lastWeightTime > 0.0 && now > g_lastWeightTime
        ? std::min(now - g_lastWeightTime, kLeadFadeSeconds)
        : 0.0;
    g_lastWeightTime = now;
    const float step = static_cast<float>(elapsed / kLeadFadeSeconds);
    const auto approach = [step](float& weight, const bool trusted) {
        weight = trusted ? std::min(1.0F, weight + step)
                         : std::max(0.0F, weight - step);
    };
    approach(g_leadWeight.x, g_trusted.yaw);
    approach(g_leadWeight.y, g_trusted.pitch);
    if (g_leadWeight.x <= 0.0F && g_leadWeight.y <= 0.0F) {
        return {};
    }

    const float progress = static_cast<float>(sample.alpha);
    mouse::Vec2 predicted = mouse::PredictAmount(
        g_record.carry,
        {g_scale * static_cast<float>(g_pendingX),
         g_scale * static_cast<float>(g_pendingY)},
        progress);
    if (g_record.invert) {
        predicted.y = -predicted.y;
    }
    mouse::LeadInput input{};
    input.lastAmount = g_record.amount;
    input.predictedAmount = predicted;
    input.progress = progress;
    input.gain = g_estimator.Gain();
    input.verified = {true, true};
    const mouse::Vec2 full = mouse::Lead(input);
    const mouse::Vec2 lead{full.x * g_leadWeight.x, full.y * g_leadWeight.y};
    if (std::abs(lead.x) > kMaximumLeadRadians ||
        std::abs(lead.y) > kMaximumLeadRadians) {
        ++g_diagnostics.rejectedLeads;
        return {};
    }
    ++g_diagnostics.leadFrames;
    g_diagnostics.maximumLeadYaw =
        std::max(g_diagnostics.maximumLeadYaw, std::abs(lead.x));
    g_diagnostics.maximumLeadPitch =
        std::max(g_diagnostics.maximumLeadPitch, std::abs(lead.y));
    return {lead.x, lead.y, true};
}

} // namespace arb::mouse_hooks
