#include "particle_hooks.h"

#include "game_build.h"
#include "logger.h"

#include <MinHook.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace arb::particle_hooks {
namespace {

// Function::particleUpdateMotion is cParticleEmitter_UserData::UpdateMotion
// (vtable +0x178): once per fixed tick it copies every live particle's
// position to its previous-position field and then advances the position.
// Function::particleUpdateGraphics is the emitter's per-viewport graphics
// update (vtable +0x68): it builds the billboards from the particle positions
// each time a viewport is drawn.
using build::Function;
constexpr std::size_t kEmitterParticlesOffset = 0x1E0U;
constexpr std::size_t kEmitterLiveCountOffset = 0x1F8U;
constexpr std::size_t kEmitterMaximumCountOffset = 0x1FCU;
constexpr std::size_t kParticlePositionOffset = 0x00U;
constexpr std::size_t kParticlePreviousPositionOffset = 0x0CU;
constexpr std::size_t kMaximumParticles = 4096U;
constexpr std::size_t kEmitterSlots = 2048U;
constexpr std::size_t kProbeLength = 8U;
// A larger single-tick move is a respawn or teleport, not motion to blend.
constexpr float kMaximumStepSquared = 25.0F;

using UpdateMotion = void(__fastcall*)(std::byte* emitter, float timeStep);
using UpdateGraphics = std::uintptr_t(__fastcall*)(std::byte* emitter,
                                                   void* frustum,
                                                   float frameTime);

struct Position {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

struct EmitterSlot {
    std::byte* emitter = nullptr;
    std::uint64_t tick = 0;
};

UpdateMotion g_originalUpdateMotion = nullptr;
UpdateGraphics g_originalUpdateGraphics = nullptr;
std::byte* g_executableBase = nullptr;
bool g_diagnosticsEnabled = false;
std::atomic<bool> g_enabled{false};
std::atomic<bool> g_renderActive{false};
std::atomic<float> g_alpha{0.0F};
std::atomic<std::uint64_t> g_tick{1};

// Which emitters advanced their particles recently. An emitter the game
// stopped updating keeps an old previous/current pair that must not be
// replayed every tick.
std::array<EmitterSlot, kEmitterSlots> g_emitters{};
std::atomic_flag g_emitterLock = ATOMIC_FLAG_INIT;

std::atomic<std::uint64_t> g_motionCalls{0};
std::atomic<std::uint64_t> g_graphicsCalls{0};
std::atomic<std::uint64_t> g_interpolatedCalls{0};
std::atomic<std::uint64_t> g_interpolatedParticles{0};
std::atomic<std::uint64_t> g_staleEmitterCalls{0};
std::atomic<std::uint64_t> g_skippedLargeSteps{0};
std::atomic<std::uint64_t> g_oversizedEmitters{0};
std::atomic<std::uint64_t> g_registryOverflows{0};
std::atomic<std::uint32_t> g_maximumLiveCount{0};
std::atomic<bool> g_loggedActive{false};
ULONGLONG g_lastReportMilliseconds = 0;
std::uint64_t g_reportTicks = 0;

std::atomic<std::uint64_t> g_ownTicks{0};
std::int64_t g_counterFrequency = 0;

thread_local std::array<Position, kMaximumParticles> t_saved;

std::int64_t CounterNow() noexcept {
    if (!g_diagnosticsEnabled || g_counterFrequency <= 0) {
        return 0;
    }
    LARGE_INTEGER counter{};
    return QueryPerformanceCounter(&counter) ? counter.QuadPart : 0;
}

template <typename T>
T ReadAt(const std::byte* const base, const std::size_t offset) noexcept {
    T result{};
    std::memcpy(&result, base + offset, sizeof(result));
    return result;
}

std::size_t SlotIndex(const std::byte* const emitter) noexcept {
    return (reinterpret_cast<std::uintptr_t>(emitter) >> 4U) % kEmitterSlots;
}

void Lock() noexcept {
    while (g_emitterLock.test_and_set(std::memory_order_acquire)) {
        YieldProcessor();
    }
}

void Unlock() noexcept {
    g_emitterLock.clear(std::memory_order_release);
}

void MarkUpdated(std::byte* const emitter) noexcept {
    const std::uint64_t tick = g_tick.load(std::memory_order_relaxed);
    const std::size_t start = SlotIndex(emitter);
    Lock();
    EmitterSlot* reusable = nullptr;
    for (std::size_t probe = 0; probe < kProbeLength; ++probe) {
        EmitterSlot& slot = g_emitters[(start + probe) % kEmitterSlots];
        if (slot.emitter == emitter) {
            slot.tick = tick;
            Unlock();
            return;
        }
        if (reusable == nullptr &&
            (slot.emitter == nullptr || slot.tick + 1U < tick)) {
            reusable = &slot;
        }
    }
    if (reusable != nullptr) {
        *reusable = {emitter, tick};
    } else {
        g_registryOverflows.fetch_add(1U, std::memory_order_relaxed);
    }
    Unlock();
}

bool UpdatedRecently(const std::byte* const emitter) noexcept {
    const std::uint64_t tick = g_tick.load(std::memory_order_relaxed);
    const std::size_t start = SlotIndex(emitter);
    bool recent = false;
    Lock();
    for (std::size_t probe = 0; probe < kProbeLength; ++probe) {
        const EmitterSlot& slot = g_emitters[(start + probe) % kEmitterSlots];
        if (slot.emitter == emitter) {
            // The particle update may run before or after the tick-complete
            // message, so either of the last two tick stamps is current.
            recent = slot.tick + 1U >= tick;
            break;
        }
    }
    Unlock();
    return recent;
}

void __fastcall UpdateMotionHook(std::byte* const emitter,
                                 const float timeStep) noexcept {
    g_originalUpdateMotion(emitter, timeStep);
    if (g_enabled.load(std::memory_order_relaxed) && emitter != nullptr) {
        g_motionCalls.fetch_add(1U, std::memory_order_relaxed);
        MarkUpdated(emitter);
    }
}

std::uintptr_t __fastcall UpdateGraphicsHook(std::byte* const emitter,
                                             void* const frustum,
                                             const float frameTime) noexcept {
    if (!g_enabled.load(std::memory_order_relaxed) || emitter == nullptr) {
        return g_originalUpdateGraphics(emitter, frustum, frameTime);
    }
    g_graphicsCalls.fetch_add(1U, std::memory_order_relaxed);
    if (!g_renderActive.load(std::memory_order_relaxed)) {
        return g_originalUpdateGraphics(emitter, frustum, frameTime);
    }
    std::byte** const particles =
        ReadAt<std::byte**>(emitter, kEmitterParticlesOffset);
    const auto liveCount = ReadAt<std::uint32_t>(emitter,
                                                 kEmitterLiveCountOffset);
    const auto maximumCount =
        ReadAt<std::uint32_t>(emitter, kEmitterMaximumCountOffset);
    if (particles == nullptr || liveCount == 0U || liveCount > maximumCount) {
        return g_originalUpdateGraphics(emitter, frustum, frameTime);
    }
    if (liveCount > kMaximumParticles) {
        g_oversizedEmitters.fetch_add(1U, std::memory_order_relaxed);
        return g_originalUpdateGraphics(emitter, frustum, frameTime);
    }
    if (!UpdatedRecently(emitter)) {
        g_staleEmitterCalls.fetch_add(1U, std::memory_order_relaxed);
        return g_originalUpdateGraphics(emitter, frustum, frameTime);
    }

    const std::int64_t started = CounterNow();
    const float alpha = g_alpha.load(std::memory_order_relaxed);
    std::array<Position, kMaximumParticles>& saved = t_saved;
    std::uint32_t largeSteps = 0;
    for (std::uint32_t index = 0; index < liveCount; ++index) {
        std::byte* const particle = particles[index];
        Position current{};
        Position previous{};
        std::memcpy(&current, particle + kParticlePositionOffset,
                    sizeof(current));
        std::memcpy(&previous, particle + kParticlePreviousPositionOffset,
                    sizeof(previous));
        saved[index] = current;
        const float dx = current.x - previous.x;
        const float dy = current.y - previous.y;
        const float dz = current.z - previous.z;
        const float stepSquared = dx * dx + dy * dy + dz * dz;
        if (!(stepSquared <= kMaximumStepSquared)) {
            // Also taken for non-finite values; the particle is left as is.
            ++largeSteps;
            continue;
        }
        const Position presented{previous.x + dx * alpha,
                                 previous.y + dy * alpha,
                                 previous.z + dz * alpha};
        std::memcpy(particle + kParticlePositionOffset, &presented,
                    sizeof(presented));
    }

    const std::int64_t substituted = CounterNow();
    const std::uintptr_t result =
        g_originalUpdateGraphics(emitter, frustum, frameTime);
    const std::int64_t drawn = CounterNow();

    // The emitter's array and live count are only changed by the fixed-tick
    // update, which cannot run inside this call.
    for (std::uint32_t index = 0; index < liveCount; ++index) {
        std::memcpy(particles[index] + kParticlePositionOffset, &saved[index],
                    sizeof(Position));
    }
    // Our own cost only: substitution plus restoration, not the native draw.
    if (started > 0) {
        const std::int64_t own =
            (substituted - started) + (CounterNow() - drawn);
        if (own > 0) {
            g_ownTicks.fetch_add(static_cast<std::uint64_t>(own),
                                 std::memory_order_relaxed);
        }
    }

    g_interpolatedCalls.fetch_add(1U, std::memory_order_relaxed);
    g_interpolatedParticles.fetch_add(liveCount - largeSteps,
                                      std::memory_order_relaxed);
    if (largeSteps != 0U) {
        g_skippedLargeSteps.fetch_add(largeSteps, std::memory_order_relaxed);
    }
    std::uint32_t maximum = g_maximumLiveCount.load(std::memory_order_relaxed);
    while (liveCount > maximum &&
           !g_maximumLiveCount.compare_exchange_weak(
               maximum, liveCount, std::memory_order_relaxed)) {
    }
    if (!g_loggedActive.exchange(true, std::memory_order_acq_rel)) {
        log::Write(log::Level::info,
                   L"Particle interpolation active on the camera presentation "
                   L"clock.");
    }
    return result;
}

void MaybeReport() {
    if (!g_diagnosticsEnabled) {
        return;
    }
    ++g_reportTicks;
    const ULONGLONG now = GetTickCount64();
    if (g_lastReportMilliseconds == 0U) {
        g_lastReportMilliseconds = now;
        return;
    }
    if (now - g_lastReportMilliseconds < 5000U) {
        return;
    }
    const auto take = [](std::atomic<std::uint64_t>& counter) {
        return std::to_wstring(
            counter.exchange(0U, std::memory_order_relaxed));
    };
    log::Write(
        log::Level::info,
        L"Particle interpolation: ticks=" + std::to_wstring(g_reportTicks) +
            L", emitter motion updates=" + take(g_motionCalls) +
            L", emitter draws total/interpolated/not-recently-updated=" +
            take(g_graphicsCalls) + L"/" + take(g_interpolatedCalls) + L"/" +
            take(g_staleEmitterCalls) + L", particles interpolated=" +
            take(g_interpolatedParticles) + L", large steps left alone=" +
            take(g_skippedLargeSteps) + L", oversized emitters=" +
            take(g_oversizedEmitters) + L", registry overflow=" +
            take(g_registryOverflows) + L", most live particles=" +
            std::to_wstring(g_maximumLiveCount.exchange(
                0U, std::memory_order_relaxed)) +
            L", own CPU total=" +
            std::to_wstring(
                g_counterFrequency <= 0
                    ? 0.0
                    : static_cast<double>(g_ownTicks.exchange(
                          0U, std::memory_order_relaxed)) *
                          1000.0 / static_cast<double>(g_counterFrequency)) +
            L" ms.");
    g_reportTicks = 0;
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
        error = L"Host module is null while installing particle hooks.";
        return false;
    }
    auto* const base = reinterpret_cast<std::byte*>(executable);
    std::byte* const motionTarget =
        base + build::Rva(Function::particleUpdateMotion);
    std::byte* const graphicsTarget =
        base + build::Rva(Function::particleUpdateGraphics);
    if (!build::EntryMatches(executable, Function::particleUpdateMotion) ||
        !build::EntryMatches(executable, Function::particleUpdateGraphics)) {
        error = L"Particle emitter entry bytes do not match the supported "
                L"build.";
        return false;
    }
    MH_STATUS status = MH_Initialize();
    if (status != MH_OK && status != MH_ERROR_ALREADY_INITIALIZED) {
        error = MinHookError(L"MH_Initialize", status);
        return false;
    }
    status = MH_CreateHook(motionTarget,
                           reinterpret_cast<void*>(&UpdateMotionHook),
                           reinterpret_cast<void**>(&g_originalUpdateMotion));
    if (status != MH_OK) {
        error = MinHookError(L"MH_CreateHook(particle motion)", status);
        return false;
    }
    status = MH_CreateHook(
        graphicsTarget, reinterpret_cast<void*>(&UpdateGraphicsHook),
        reinterpret_cast<void**>(&g_originalUpdateGraphics));
    if (status != MH_OK) {
        static_cast<void>(MH_RemoveHook(motionTarget));
        g_originalUpdateMotion = nullptr;
        error = MinHookError(L"MH_CreateHook(particle graphics)", status);
        return false;
    }
    g_executableBase = base;
    g_diagnosticsEnabled = diagnosticsEnabled;
    LARGE_INTEGER frequency{};
    g_counterFrequency =
        QueryPerformanceFrequency(&frequency) ? frequency.QuadPart : 0;
    g_enabled.store(true, std::memory_order_release);
    status = MH_EnableHook(motionTarget);
    if (status == MH_OK) {
        status = MH_EnableHook(graphicsTarget);
    }
    if (status != MH_OK) {
        Stop();
        error = MinHookError(L"MH_EnableHook(particles)", status);
        return false;
    }
    return true;
}

void Stop() noexcept {
    g_enabled.store(false, std::memory_order_release);
    g_renderActive.store(false, std::memory_order_release);
    if (g_executableBase != nullptr) {
        for (const Function function : {Function::particleUpdateGraphics,
                                        Function::particleUpdateMotion}) {
            const std::uintptr_t rva = build::Rva(function);
            static_cast<void>(MH_DisableHook(g_executableBase + rva));
            static_cast<void>(MH_RemoveHook(g_executableBase + rva));
        }
    }
    g_originalUpdateMotion = nullptr;
    g_originalUpdateGraphics = nullptr;
    g_executableBase = nullptr;
    Lock();
    g_emitters = {};
    Unlock();
}

void OnTickCompleted() noexcept {
    if (!g_enabled.load(std::memory_order_relaxed)) {
        return;
    }
    g_tick.fetch_add(1U, std::memory_order_relaxed);
    MaybeReport();
}

void BeginRender(const clock::Sample& sample) noexcept {
    if (!g_enabled.load(std::memory_order_relaxed) ||
        sample.kind != clock::SampleKind::interpolated ||
        !std::isfinite(sample.alpha) || sample.alpha < 0.0 ||
        sample.alpha > 1.0) {
        return;
    }
    g_alpha.store(static_cast<float>(sample.alpha),
                  std::memory_order_relaxed);
    g_renderActive.store(true, std::memory_order_release);
}

void EndRender() noexcept {
    g_renderActive.store(false, std::memory_order_release);
}

} // namespace arb::particle_hooks
