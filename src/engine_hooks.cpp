#include "engine_hooks.h"

#include "host_validation.h"
#include "logger.h"

#include <MinHook.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace arb::engine {
namespace {

constexpr std::uintptr_t kEngineRunRva = 0x00318130U;
constexpr std::size_t kLimitFpsOffset = 0x88U;
constexpr std::array<std::uint8_t, 24> kEngineRunEntry{
    0x40, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41,
    0x56, 0x48, 0x8D, 0x6C, 0x24, 0xD8, 0x48, 0x81,
    0xEC, 0x28, 0x01, 0x00, 0x00, 0x48, 0x8B, 0xF1,
};

using EngineRun = void(__fastcall*)(std::byte* engine);

EngineRun g_originalEngineRun = nullptr;
std::atomic<std::byte*> g_engineInstance{nullptr};
std::atomic<bool> g_loggedActivation{false};
std::atomic<bool> g_loggedInvalidFlag{false};

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

void __fastcall EngineRunHook(std::byte* const engine) noexcept {
    g_engineInstance.store(engine, std::memory_order_release);
    if (engine != nullptr) {
        auto* const limitFps = reinterpret_cast<std::uint8_t*>(
            engine + kLimitFpsOffset);
        const std::uint8_t observed = *limitFps;
        if (observed <= 1U) {
            *limitFps = 0;
            if (!g_loggedActivation.exchange(true, std::memory_order_acq_rel)) {
                log::Write(log::Level::info,
                           L"Render unlock active: validated "
                           L"cEngine::mbLimitFPS and set it to false.");
            }
        } else if (!g_loggedInvalidFlag.exchange(true,
                                                 std::memory_order_acq_rel)) {
            log::Write(log::Level::error,
                       L"Render unlock disabled at runtime: the byte at "
                       L"cEngine+0x88 was not boolean.");
        }
    }
    g_originalEngineRun(engine);
    g_engineInstance.store(nullptr, std::memory_order_release);
}

} // namespace

bool StartRenderUnlock(const HMODULE executable, std::wstring& error) noexcept {
    if (executable == nullptr) {
        error = L"Host module is null while installing render unlock.";
        return false;
    }
    auto* const target = reinterpret_cast<std::byte*>(executable) + kEngineRunRva;
    if (!host::AddressMatches(target, kEngineRunEntry.data(), kEngineRunEntry.size())) {
        error = L"cEngine::Run entry bytes do not match the supported build.";
        return false;
    }

    MH_STATUS status = MH_Initialize();
    if (status != MH_OK && status != MH_ERROR_ALREADY_INITIALIZED) {
        error = MinHookError(L"MH_Initialize", status);
        return false;
    }
    status = MH_CreateHook(target, reinterpret_cast<void*>(&EngineRunHook),
                           reinterpret_cast<void**>(&g_originalEngineRun));
    if (status != MH_OK) {
        error = MinHookError(L"MH_CreateHook(cEngine::Run)", status);
        return false;
    }
    status = MH_EnableHook(target);
    if (status != MH_OK) {
        MH_RemoveHook(target);
        g_originalEngineRun = nullptr;
        error = MinHookError(L"MH_EnableHook(cEngine::Run)", status);
        return false;
    }
    return true;
}

std::byte* Instance() noexcept {
    return g_engineInstance.load(std::memory_order_acquire);
}

void StopRenderUnlock() noexcept {
    const HMODULE executable = GetModuleHandleW(nullptr);
    if (executable == nullptr) {
        return;
    }
    auto* const target =
        reinterpret_cast<std::byte*>(executable) + kEngineRunRva;
    static_cast<void>(MH_DisableHook(target));
    static_cast<void>(MH_RemoveHook(target));
    g_originalEngineRun = nullptr;
    g_engineInstance.store(nullptr, std::memory_order_release);
}

} // namespace arb::engine
