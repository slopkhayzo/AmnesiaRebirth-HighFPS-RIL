#include "bootstrap.h"

#include "camera_hooks.h"
#include "config.h"
#include "engine_hooks.h"
#include "game_build.h"
#include "gui_cursor.h"
#include "host_validation.h"
#include "logger.h"
#include "mouse_hooks.h"
#include "particle_hooks.h"
#include "plugin_version.h"
#include "root_probe.h"
#include "swap_telemetry.h"

#include <Windows.h>

#include <algorithm>
#include <atomic>
#include <string>
#include <string_view>
#include <vector>

namespace arb {
namespace {

enum class BootstrapState : unsigned int {
    dormant,
    starting,
    disabled,
    rejected,
    readyWithoutHooks,
    readyWithHooks,
};

std::atomic<BootstrapState> g_state{BootstrapState::dormant};

std::wstring ModulePath(const HMODULE module) {
    std::vector<wchar_t> buffer(512);
    for (;;) {
        const DWORD copied = GetModuleFileNameW(
            module, buffer.data(), static_cast<DWORD>(buffer.size()));
        if (copied == 0) {
            return {};
        }
        if (copied < buffer.size() - 1) {
            return std::wstring(buffer.data(), copied);
        }
        if (buffer.size() >= 32768) {
            return {};
        }
        buffer.resize(std::min<std::size_t>(buffer.size() * 2, 32768));
    }
}

std::wstring ReplaceExtension(const std::wstring& path,
                              const std::wstring_view extension) {
    const std::size_t slash = path.find_last_of(L"\\/");
    const std::size_t dot = path.find_last_of(L'.');
    if (dot == std::wstring::npos ||
        (slash != std::wstring::npos && dot < slash)) {
        return path + std::wstring(extension);
    }
    return path.substr(0, dot) + std::wstring(extension);
}

std::wstring LocalAppDataLogPath() {
    std::vector<wchar_t> buffer(32768);
    const DWORD copied = GetEnvironmentVariableW(
        L"LOCALAPPDATA", buffer.data(), static_cast<DWORD>(buffer.size()));
    if (copied == 0 || copied >= buffer.size()) {
        return {};
    }
    std::wstring directory(buffer.data(), copied);
    directory.append(L"\\AmnesiaRebirth.FPSFix");
    if (!CreateDirectoryW(directory.c_str(), nullptr) &&
        GetLastError() != ERROR_ALREADY_EXISTS) {
        return {};
    }
    return directory + L"\\AmnesiaRebirth.FPSFix.log";
}

std::wstring Hex(const std::uint64_t value) {
    wchar_t buffer[32]{};
    _snwprintf_s(buffer, std::size(buffer), _TRUNCATE, L"0x%llX",
                 static_cast<unsigned long long>(value));
    return buffer;
}

} // namespace

DWORD WINAPI BootstrapThread(void* moduleParameter) {
    g_state.store(BootstrapState::starting, std::memory_order_release);
    const HMODULE pluginModule = static_cast<HMODULE>(moduleParameter);
    const std::wstring pluginPath = ModulePath(pluginModule);
    if (pluginPath.empty()) {
        OutputDebugStringW(
            L"[AmnesiaRebirth.FPSFix] Could not resolve plugin path.\n");
        g_state.store(BootstrapState::rejected, std::memory_order_release);
        return 1;
    }

    const std::wstring iniPath = ReplaceExtension(pluginPath, L".ini");
    const std::wstring logPath = ReplaceExtension(pluginPath, L".log");
    // Each launch starts a fresh log; the last session is kept alongside.
    static_cast<void>(MoveFileExW(
        logPath.c_str(),
        ReplaceExtension(pluginPath, L".previous.log").c_str(),
        MOVEFILE_REPLACE_EXISTING));
    if (!log::Initialize(logPath)) {
        const std::wstring fallback = LocalAppDataLogPath();
        if (fallback.empty() || !log::Initialize(fallback)) {
            OutputDebugStringW(
                L"[AmnesiaRebirth.FPSFix] Could not open a log file.\n");
        } else {
            log::Write(log::Level::warning,
                       L"Could not write sibling log; using: " + fallback);
        }
    }

    log::Write(log::Level::info,
               std::wstring(AMNESIA_REBIRTH_FPS_FIX_NAME) + L" " +
                   AMNESIA_REBIRTH_FPS_FIX_VERSION +
                   L" bootstrap starting (x64)");
    log::Write(log::Level::debug, L"Plugin path: " + pluginPath);
    log::Write(log::Level::debug, L"INI path: " + iniPath);

    const config::LoadResult loaded = config::Load(iniPath);
    if (!loaded.ok) {
        log::Write(log::Level::error, loaded.error);
        log::Write(log::Level::error,
                   L"Fail-closed: no engine hooks were installed.");
        g_state.store(BootstrapState::rejected, std::memory_order_release);
        return 2;
    }
    log::SetDiagnosticsEnabled(loaded.settings.diagnostics);
    if (!loaded.settings.enabled) {
        log::Write(log::Level::info, L"Plugin disabled by configuration.");
        g_state.store(BootstrapState::disabled, std::memory_order_release);
        return 0;
    }

    const host::ValidationResult validation = host::Validate(GetModuleHandleW(nullptr));
    if (!validation.compatible) {
        log::Write(log::Level::error, L"Host validation failed: " + validation.error);
        log::Write(log::Level::error,
                   L"Fail-closed: no engine hooks were installed.");
        g_state.store(BootstrapState::rejected, std::memory_order_release);
        return 3;
    }
    log::Write(log::Level::info,
               L"Eligible Rebirth PE64 host: machine=" + Hex(validation.machine) +
                   L", imageSize=" + Hex(validation.imageSize) +
                   L", timestamp=" + Hex(validation.timestamp));
    const build::Build* const gameBuild =
        build::Select(GetModuleHandleW(nullptr));
    if (gameBuild == nullptr) {
        log::Write(log::Level::warning,
                   L"No supported build matched this executable's "
                   L"cEngine::Run; every feature will refuse to start.");
    } else {
        const bool sameIdentity =
            gameBuild->timestamp == validation.timestamp &&
            gameBuild->imageSize == validation.imageSize;
        log::Write(log::Level::info,
                   std::wstring(L"Game build: ") + gameBuild->name +
                       (sameIdentity
                            ? L" (exact executable the mapping was made from)."
                            : L" (code matches; timestamp or image size "
                              L"differ, so each feature still checks its "
                              L"game code)."));
    }

    config::Features unsupportedFeatures = loaded.settings.features;
    unsupportedFeatures.renderUnlock = false;
    unsupportedFeatures.presentationLimiter = false;
    unsupportedFeatures.cameraInterpolation = false;
    unsupportedFeatures.handsRootInterpolation = false;
    unsupportedFeatures.worldRootInterpolation = false;
    unsupportedFeatures.handsPoseInterpolation = false;
    unsupportedFeatures.ragdollPoseInterpolation = false;
    unsupportedFeatures.mouseOverlay = false;
    unsupportedFeatures.particleInterpolation = false;
    if (config::AnyBehavioralFeatureEnabled(unsupportedFeatures)) {
        log::Write(log::Level::error,
                   L"A behavioral feature was requested before its Rebirth "
                   L"mapping was implemented.");
        log::Write(log::Level::error,
                   L"Fail-closed: no engine hooks were installed.");
        g_state.store(BootstrapState::rejected, std::memory_order_release);
        return 4;
    }

    if ((loaded.settings.features.presentationLimiter ||
         loaded.settings.features.cameraInterpolation ||
         loaded.settings.features.handsRootInterpolation ||
         loaded.settings.features.worldRootInterpolation ||
         loaded.settings.features.handsPoseInterpolation ||
         loaded.settings.features.ragdollPoseInterpolation) &&
        !loaded.settings.features.renderUnlock) {
        log::Write(log::Level::error,
                   L"PresentationLimiter, CameraInterpolation, and "
                   L"hands/body/world interpolation require "
                   L"RenderUnlock=true.");
        g_state.store(BootstrapState::rejected, std::memory_order_release);
        return 5;
    }
    if (loaded.settings.features.handsRootInterpolation &&
        !loaded.settings.features.cameraInterpolation) {
        log::Write(log::Level::error,
                   L"HandsRootInterpolation requires "
                   L"CameraInterpolation=true.");
        g_state.store(BootstrapState::rejected, std::memory_order_release);
        return 5;
    }
    if (loaded.settings.features.worldRootInterpolation &&
        !loaded.settings.features.cameraInterpolation) {
        log::Write(log::Level::error,
                   L"WorldRootInterpolation requires "
                   L"CameraInterpolation=true.");
        g_state.store(BootstrapState::rejected, std::memory_order_release);
        return 5;
    }
    if (loaded.settings.features.handsPoseInterpolation &&
        (!loaded.settings.features.cameraInterpolation ||
         !loaded.settings.features.handsRootInterpolation)) {
        log::Write(log::Level::error,
                   L"HandsPoseInterpolation requires CameraInterpolation=true "
                   L"and HandsRootInterpolation=true.");
        g_state.store(BootstrapState::rejected, std::memory_order_release);
        return 5;
    }
    if (loaded.settings.features.ragdollPoseInterpolation &&
        (!loaded.settings.features.cameraInterpolation ||
         !loaded.settings.features.worldRootInterpolation)) {
        log::Write(log::Level::error,
                   L"RagdollPoseInterpolation requires "
                   L"CameraInterpolation=true and "
                   L"WorldRootInterpolation=true.");
        g_state.store(BootstrapState::rejected, std::memory_order_release);
        return 5;
    }
    if (loaded.settings.features.mouseOverlay &&
        !loaded.settings.features.cameraInterpolation) {
        log::Write(log::Level::error,
                   L"MouseOverlay requires CameraInterpolation=true.");
        g_state.store(BootstrapState::rejected, std::memory_order_release);
        return 5;
    }
    if (loaded.settings.features.presentationLimiter &&
        loaded.settings.targetFps == 0U) {
        log::Write(log::Level::error,
                   L"PresentationLimiter requires TargetFPS greater than zero.");
        g_state.store(BootstrapState::rejected, std::memory_order_release);
        return 6;
    }

    if (loaded.settings.motionTraceEnabled) {
        log::Write(log::Level::warning,
                   L"MotionTrace is not available before timing hooks exist.");
    }
    if (loaded.settings.telemetryEnabled ||
        loaded.settings.features.presentationLimiter) {
        std::wstring swapError;
        if (!telemetry::StartSwapRuntime(
                GetModuleHandleW(nullptr), loaded.settings.telemetryEnabled,
                loaded.settings.telemetryReportIntervalSeconds,
                loaded.settings.features.presentationLimiter,
                loaded.settings.targetFps, swapError)) {
            log::Write(log::Level::error,
                       L"SDL swap runtime could not start: " + swapError);
            log::Write(log::Level::error,
                       L"Fail-closed: no behavioral hooks were installed.");
            g_state.store(BootstrapState::rejected, std::memory_order_release);
            return 7;
        }
        if (loaded.settings.telemetryEnabled) {
            log::Write(log::Level::info,
                       L"SDL_GL_SwapWindow cadence telemetry installed.");
        }
        if (loaded.settings.features.presentationLimiter) {
            log::Write(log::Level::info,
                       L"Presentation limiter installed at " +
                           std::to_wstring(loaded.settings.targetFps) + L" FPS.");
        }
    }

    if (loaded.settings.features.renderUnlock) {
        std::wstring hookError;
        if (!engine::StartRenderUnlock(GetModuleHandleW(nullptr), hookError)) {
            telemetry::StopSwapRuntime();
            log::Write(log::Level::error,
                       L"Render unlock could not start: " + hookError);
            log::Write(log::Level::error,
                       L"Fail-closed: stock cEngine::mbLimitFPS was not changed.");
            g_state.store(BootstrapState::rejected, std::memory_order_release);
            return 8;
        }
        log::Write(log::Level::info,
                   L"Guarded cEngine::Run render-unlock detour installed.");
    }

    if (loaded.settings.features.cameraInterpolation) {
        std::wstring cameraError;
        if (!camera_hooks::Start(GetModuleHandleW(nullptr), cameraError)) {
            engine::StopRenderUnlock();
            telemetry::StopSwapRuntime();
            log::Write(log::Level::error,
                       L"Camera interpolation could not start: " + cameraError);
            log::Write(log::Level::error,
                       L"Fail-closed: render unlock was rolled back.");
            g_state.store(BootstrapState::rejected, std::memory_order_release);
            return 9;
        }
        log::Write(log::Level::info,
                   L"Guarded updater and listener-viewport interpolation "
                   L"detours installed.");

        if (loaded.settings.diagnostics ||
            loaded.settings.features.handsRootInterpolation ||
            loaded.settings.features.worldRootInterpolation ||
            loaded.settings.features.handsPoseInterpolation ||
            loaded.settings.features.ragdollPoseInterpolation) {
            std::wstring probeError;
            if (root_probe::Start(
                    GetModuleHandleW(nullptr),
                    loaded.settings.features.handsRootInterpolation,
                    loaded.settings.features.handsPoseInterpolation,
                    loaded.settings.features.worldRootInterpolation,
                    loaded.settings.features.ragdollPoseInterpolation,
                    loaded.settings.diagnostics, probeError)) {
                std::wstring mode =
                    loaded.settings.features.handsRootInterpolation
                        ? L"Render-only PlayerHands/PlayerBody root "
                          L"interpolation"
                        : L"Read-only PlayerHands/PlayerBody root probe";
                if (loaded.settings.features.handsPoseInterpolation) {
                    mode += L" plus guarded PlayerBody and selected scripted "
                            L"entity skeletal-pose interpolation";
                }
                if (loaded.settings.features.worldRootInterpolation) {
                    mode += L" plus bounded Newton rigid-body world-root "
                            L"interpolation";
                }
                if (loaded.settings.features.ragdollPoseInterpolation) {
                    mode += L" plus guarded skeletal prop pose "
                            L"interpolation";
                }
                log::Write(log::Level::info, mode + L" runtime installed.");
            } else if (loaded.settings.features.handsRootInterpolation ||
                       loaded.settings.features.worldRootInterpolation ||
                       loaded.settings.features.handsPoseInterpolation ||
                       loaded.settings.features.ragdollPoseInterpolation) {
                camera_hooks::Stop();
                engine::StopRenderUnlock();
                telemetry::StopSwapRuntime();
                log::Write(log::Level::error,
                           L"Player interpolation could not start: " +
                               probeError);
                log::Write(log::Level::error,
                           L"Fail-closed: behavioral hooks were rolled back.");
                g_state.store(BootstrapState::rejected,
                              std::memory_order_release);
                return 10;
            } else {
                log::Write(log::Level::warning,
                           L"Root probe was unavailable: " + probeError);
            }
        }

        if (loaded.settings.features.mouseOverlay) {
            // The preview only adds to camera interpolation, so a mapping
            // failure leaves the rest running with stock mouse behavior.
            std::wstring mouseError;
            if (mouse_hooks::Start(GetModuleHandleW(nullptr),
                                   loaded.settings.diagnostics, mouseError)) {
                log::Write(log::Level::info,
                           L"Gameplay mouse-look preview installed: SDL "
                           L"relative-motion read and look-amount detours.");
            } else {
                log::Write(log::Level::error,
                           L"Mouse-look preview could not start and stays "
                           L"off: " + mouseError);
            }
            std::wstring cursorError;
            if (gui_cursor::Start(GetModuleHandleW(nullptr),
                                  loaded.settings.diagnostics, cursorError)) {
                log::Write(log::Level::info,
                           L"GUI cursor preview installed: GUI "
                           L"mouse-position detour.");
            } else {
                log::Write(log::Level::error,
                           L"GUI cursor preview could not start and stays "
                           L"off: " + cursorError);
            }
        }

        if (loaded.settings.features.particleInterpolation) {
            // Purely additive as well: on a mapping failure particles keep
            // their stock fixed-tick motion.
            std::wstring particleError;
            if (particle_hooks::Start(GetModuleHandleW(nullptr),
                                      loaded.settings.diagnostics,
                                      particleError)) {
                log::Write(log::Level::info,
                           L"Particle interpolation installed: emitter "
                           L"motion and per-viewport graphics detours.");
            } else {
                log::Write(log::Level::error,
                           L"Particle interpolation could not start and "
                           L"stays off: " + particleError);
            }
        }
    }

    if (loaded.settings.telemetryEnabled ||
        config::AnyBehavioralFeatureEnabled(loaded.settings.features)) {
        log::Write(log::Level::info,
                   L"Bootstrap complete with requested hooks enabled.");
        g_state.store(BootstrapState::readyWithHooks, std::memory_order_release);
    } else {
        log::Write(log::Level::info,
                   loaded.settings.features.renderUnlock
                       ? L"Bootstrap complete with render unlock enabled."
                       : L"Bootstrap complete. No hooks installed.");
        g_state.store(BootstrapState::readyWithoutHooks, std::memory_order_release);
    }
    return 0;
}

} // namespace arb

