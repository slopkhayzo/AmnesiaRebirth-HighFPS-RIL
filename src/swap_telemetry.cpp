#include "swap_telemetry.h"

#include "frame_cadence.h"
#include "frame_limiter.h"
#include "iat_hook.h"
#include "logger.h"

#include <Windows.h>

#include <atomic>
#include <cstdio>
#include <string>

namespace arb::telemetry {
namespace {

using SwapWindow = void(__cdecl*)(void* window);

std::atomic<SwapWindow> g_originalSwap{nullptr};
void** g_swapSlot = nullptr;
cadence::FrameCadence g_cadence;
limiter::DeadlinePacer g_limiter;
std::int64_t g_frequency = 0;
unsigned int g_reportIntervalSeconds = 5;
bool g_telemetryEnabled = false;
bool g_limiterEnabled = false;
std::atomic<bool> g_pacedBeforeRender{false};
// Sample-to-present delay of frames paced before rendering, in QPC ticks.
std::atomic<std::int64_t> g_sampleCounter{0};
std::atomic<std::int64_t> g_latencySum{0};
std::atomic<std::int64_t> g_latencyMinimum{0};
std::atomic<std::int64_t> g_latencyMaximum{0};
std::atomic<std::uint32_t> g_latencySamples{0};

std::wstring FormatSnapshot(const cadence::Snapshot& snapshot) {
    const double framesPerSecond = snapshot.meanMilliseconds > 0.0
                                       ? 1000.0 / snapshot.meanMilliseconds
                                       : 0.0;
    wchar_t message[512]{};
    _snwprintf_s(
        message, std::size(message), _TRUNCATE,
        L"Swap cadence: samples=%llu mean=%.3f ms (%.1f FPS), p50=%.3f ms, "
        L"p95=%.3f ms, p99=%.3f ms, max=%.3f ms, 100-250ms=%u, >250ms=%u, "
        L"invalid=%u",
        static_cast<unsigned long long>(snapshot.samples),
        snapshot.meanMilliseconds, framesPerSecond, snapshot.p50Milliseconds,
        snapshot.p95Milliseconds, snapshot.p99Milliseconds,
        snapshot.maxMilliseconds, snapshot.overflow, snapshot.longGaps,
        snapshot.invalid);
    return message;
}

double CounterSeconds(const std::int64_t counter) noexcept {
    return static_cast<double>(counter) / static_cast<double>(g_frequency);
}

void WaitUntil(const double deadlineSeconds) noexcept {
    for (;;) {
        LARGE_INTEGER counter{};
        if (!QueryPerformanceCounter(&counter)) {
            return;
        }
        const double remaining = deadlineSeconds - CounterSeconds(counter.QuadPart);
        if (remaining <= 0.0) {
            return;
        }
        if (remaining > 0.002) {
            const DWORD sleepMilliseconds = static_cast<DWORD>(
                (remaining - 0.001) * 1000.0);
            if (sleepMilliseconds != 0U) {
                Sleep(sleepMilliseconds);
                continue;
            }
        }
        if (!SwitchToThread()) {
            YieldProcessor();
        }
    }
}

void __cdecl SwapWindowHook(void* const window) noexcept {
    LARGE_INTEGER counter{};
    if (QueryPerformanceCounter(&counter)) {
        // Frames that already waited before sampling their scene present
        // immediately; anything else (menus, loading) is paced here.
        const bool alreadyPaced =
            g_pacedBeforeRender.exchange(false, std::memory_order_acq_rel);
        if (alreadyPaced && g_telemetryEnabled) {
            const std::int64_t latency = counter.QuadPart -
                g_sampleCounter.load(std::memory_order_acquire);
            if (latency >= 0) {
                const std::uint32_t samples = g_latencySamples.fetch_add(
                    1U, std::memory_order_acq_rel);
                g_latencySum.fetch_add(latency, std::memory_order_acq_rel);
                if (samples == 0U ||
                    latency < g_latencyMinimum.load(std::memory_order_acquire)) {
                    g_latencyMinimum.store(latency, std::memory_order_release);
                }
                if (samples == 0U ||
                    latency > g_latencyMaximum.load(std::memory_order_acquire)) {
                    g_latencyMaximum.store(latency, std::memory_order_release);
                }
            }
        }
        if (g_limiterEnabled && !alreadyPaced) {
            const double deadline =
                g_limiter.NextDeadline(CounterSeconds(counter.QuadPart));
            WaitUntil(deadline);
            static_cast<void>(QueryPerformanceCounter(&counter));
        }
        if (g_telemetryEnabled) {
            g_cadence.RecordQpc(counter.QuadPart, g_frequency);
        }
    }
    const SwapWindow original = g_originalSwap.load(std::memory_order_acquire);
    if (original != nullptr) {
        original(window);
    }
}

DWORD WINAPI ReporterThread(void*) {
    const DWORD delay = g_reportIntervalSeconds * 1000U;
    for (;;) {
        Sleep(delay);
        const cadence::Snapshot snapshot = g_cadence.TakeSnapshot();
        if (snapshot.samples != 0 || snapshot.longGaps != 0 ||
            snapshot.invalid != 0) {
            log::Write(log::Level::info, FormatSnapshot(snapshot));
        }
        const std::uint32_t latencySamples =
            g_latencySamples.exchange(0U, std::memory_order_acq_rel);
        const std::int64_t latencySum =
            g_latencySum.exchange(0, std::memory_order_acq_rel);
        if (latencySamples != 0U && g_frequency > 0) {
            const double toMilliseconds =
                1000.0 / static_cast<double>(g_frequency);
            wchar_t message[256]{};
            _snwprintf_s(
                message, std::size(message), _TRUNCATE,
                L"Sample-to-present delay: frames=%u mean=%.3f ms, "
                L"min=%.3f ms, max=%.3f ms",
                latencySamples,
                static_cast<double>(latencySum) * toMilliseconds /
                    static_cast<double>(latencySamples),
                static_cast<double>(g_latencyMinimum.load(
                    std::memory_order_acquire)) * toMilliseconds,
                static_cast<double>(g_latencyMaximum.load(
                    std::memory_order_acquire)) * toMilliseconds);
            log::Write(log::Level::info, message);
        }
    }
}

} // namespace

bool StartSwapRuntime(const HMODULE executable, const bool telemetryEnabled,
                      const unsigned int reportIntervalSeconds,
                      const bool limiterEnabled, const unsigned int targetFps,
                      std::wstring& error) noexcept {
    LARGE_INTEGER frequency{};
    if (!QueryPerformanceFrequency(&frequency) || frequency.QuadPart <= 0) {
        error = L"QueryPerformanceFrequency failed.";
        return false;
    }
    if (limiterEnabled && !g_limiter.Configure(targetFps)) {
        error = L"PresentationLimiter requires TargetFPS in the range 1..1000.";
        return false;
    }
    const iat::LocateResult located =
        iat::Locate(executable, "SDL2.dll", "SDL_GL_SwapWindow");
    if (located.slot == nullptr || located.current == nullptr) {
        error = L"Could not locate SDL_GL_SwapWindow: " + located.error;
        return false;
    }

    g_frequency = frequency.QuadPart;
    g_reportIntervalSeconds = reportIntervalSeconds;
    g_telemetryEnabled = telemetryEnabled;
    g_limiterEnabled = limiterEnabled;
    g_originalSwap.store(reinterpret_cast<SwapWindow>(located.current),
                         std::memory_order_release);
    if (!iat::Replace(located.slot, located.current,
                      reinterpret_cast<void*>(&SwapWindowHook), error)) {
        g_originalSwap.store(nullptr, std::memory_order_release);
        return false;
    }
    g_swapSlot = located.slot;
    if (!error.empty()) {
        log::Write(log::Level::warning, error);
        error.clear();
    }

    if (telemetryEnabled) {
        const HANDLE reporter =
            CreateThread(nullptr, 0, ReporterThread, nullptr, 0, nullptr);
        if (reporter == nullptr) {
            error = L"Swap hook installed, but the telemetry reporter thread "
                    L"could not start.";
            return false;
        }
        CloseHandle(reporter);
    }
    return true;
}

void PaceBeforeRender() noexcept {
    if (!g_limiterEnabled || g_frequency <= 0 ||
        g_pacedBeforeRender.load(std::memory_order_acquire)) {
        return;
    }
    LARGE_INTEGER counter{};
    if (!QueryPerformanceCounter(&counter)) {
        return;
    }
    WaitUntil(g_limiter.NextDeadline(CounterSeconds(counter.QuadPart)));
    if (QueryPerformanceCounter(&counter)) {
        g_sampleCounter.store(counter.QuadPart, std::memory_order_release);
    }
    g_pacedBeforeRender.store(true, std::memory_order_release);
}

void StopSwapRuntime() noexcept {
    const SwapWindow original =
        g_originalSwap.exchange(nullptr, std::memory_order_acq_rel);
    g_telemetryEnabled = false;
    g_limiterEnabled = false;
    g_limiter.Reset();
    if (g_swapSlot != nullptr && original != nullptr) {
        std::wstring ignored;
        static_cast<void>(iat::Replace(
            g_swapSlot, reinterpret_cast<void*>(&SwapWindowHook),
            reinterpret_cast<void*>(original), ignored));
    }
    g_swapSlot = nullptr;
}

} // namespace arb::telemetry
