#include "gui_cursor.h"

#include "game_build.h"
#include "iat_hook.h"
#include "logger.h"

#include <MinHook.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

namespace arb::gui_cursor {
namespace {

using build::Function;

// cImGui::SendMousePosition(const cVector2l& pos, const cVector2l& rel):
// once per fixed tick the input handler passes the mouse's window position
// in pixels; the function converts it to GUI coordinates and stores it.
constexpr std::size_t kImGuiMousePositionOffset = 0x4E48U;
// Previous position (8 bytes) and relative motion (8 bytes) follow the
// position; the first-update flag sits at +0x4E7E.
constexpr std::size_t kImGuiPreviousPositionOffset = 0x4E50U;
constexpr std::size_t kImGuiFirstUpdateFlagOffset = 0x4E7EU;
constexpr std::size_t kPreservedBytes = 16U;
// The OS cursor is trusted as the game's pixel position only after the two
// have been seen to agree this closely at a tick. That is a property of the
// window, so one agreement (any tick with a resting mouse) is kept.
constexpr int kAgreementPixels = 3;


struct Pixels {
    int x = 0;
    int y = 0;
};

using SendMousePosition = void(__fastcall*)(std::byte* imGui,
                                            const Pixels* position,
                                            const Pixels* relative);
using GetRelativeMouseMode = int(__cdecl*)();

struct Diagnostics {
    std::uint64_t ticks = 0;
    std::uint64_t feeds = 0;
    std::uint64_t frames = 0;
    std::uint64_t applied = 0;
    std::uint64_t relativeModeFrames = 0;
    std::uint64_t noFeedFrames = 0;
    std::uint64_t untrustedFrames = 0;
    std::uint64_t noCursorFrames = 0;
    std::uint64_t outsideFrames = 0;
    std::uint64_t agreeingTicks = 0;
    int largestTickDifference = 0;
    int largestFrameStep = 0;
    std::uint64_t cursorDraws = 0;
    std::uint64_t cursorDrawsRefreshed = 0;
    std::uint64_t anchoredCommands = 0;
    std::uint64_t commandMoves = 0;
    std::uint64_t anchorMismatches = 0;
    std::uint64_t buildRestarts = 0;
    std::uint64_t followingCommands = 0;
    std::uint64_t followMoves = 0;
};

// cImGui image draw: appends one 0xA0-byte command to the vector at
// +0x300/+0x308; the command's position is three floats at +0x08.
constexpr std::size_t kImGuiCommandsBeginOffset = 0x300U;
constexpr std::size_t kImGuiCommandsEndOffset = 0x308U;
constexpr std::size_t kCommandSize = 0xA0U;
constexpr std::size_t kCommandPositionOffset = 0x08U;
constexpr std::size_t kGraphicOffsetOffset = 0x24U;
using DrawImage = void(__fastcall*)(std::byte* imGui, void* graphic,
                                    float* position, float* size, void* a,
                                    void* b, void* c, void* d, void* e);
struct AnchoredCommand {
    std::size_t index = 0;
    float baseX = 0.0F;
    float baseY = 0.0F;
    float writtenX = 0.0F;
    float writtenY = 0.0F;
    float anchorX = 0.0F;
    float anchorY = 0.0F;
};
DrawImage g_originalDrawImage = nullptr;
std::array<AnchoredCommand, 8> g_anchored{};
std::size_t g_anchoredCount = 0;
std::byte* g_anchoredImGui = nullptr;

constexpr std::size_t kMaximumCommands = 2048U;
struct BuildSnapshot {
    bool valid = false;
    std::byte* imGui = nullptr;
    std::size_t count = 0;
    float mouseX = 0.0F;
    float mouseY = 0.0F;
    std::array<float, kMaximumCommands> x{};
    std::array<float, kMaximumCommands> y{};
};
// One followable draw list inside cImGui: where its vector lives, how big a
// command is, and where a command keeps its position.
struct CommandList {
    std::size_t beginOffset = 0;
    std::size_t endOffset = 0;
    std::size_t size = 0;
    std::size_t positionOffset = 0;
    bool holdsCursor = false;
};
// Images (0xA0-byte commands at +0x300, position at +0x08) and text
// (0x58-byte commands at +0x318, position at +0x28, appended by GOG RVA
// 0x42D6E0, Steam RVA 0x43C510).
constexpr CommandList kImageList{0x300U, 0x308U, 0xA0U, 0x08U, true};
constexpr CommandList kTextList{0x318U, 0x320U, 0x58U, 0x28U, false};
struct FollowState {
    std::array<BuildSnapshot, 2> builds{};
    BuildSnapshot* current = &builds[0];
    BuildSnapshot* previous = &builds[1];
    std::array<bool, kMaximumCommands> following{};
    std::array<float, kMaximumCommands> writtenX{};
    std::array<float, kMaximumCommands> writtenY{};
};
FollowState g_imageFollow;
FollowState g_textFollow;
std::size_t g_lastImageIndex = 0;
bool g_lastImageIndexValid = false;
std::uint64_t g_buildSerial = 0;
std::uint64_t g_snapshotSerial = 0;

using DoMouse = void(__fastcall*)(std::byte* imGui, void* graphic,
                                  float* offset, void* size);
// cImGui::DoMouse (Function::doMouse) queues the cursor graphic.
DoMouse g_originalDoMouse = nullptr;
std::byte* g_refreshedImGui = nullptr;
std::uint64_t g_refreshedTick = 0;

SendMousePosition g_originalSendMousePosition = nullptr;
GetRelativeMouseMode g_getRelativeMouseMode = nullptr;
std::byte* g_executableBase = nullptr;
bool g_enabled = false;
bool g_diagnosticsEnabled = false;
bool g_applying = false;
DWORD g_gameThread = 0;
std::uint64_t g_tick = 1;
std::byte* g_fedImGui = nullptr;
std::uint64_t g_fedTick = 0;
Pixels g_fedPixels{};
bool g_hasAgreement = false;
bool g_loggedActive = false;
Diagnostics g_diagnostics{};
ULONGLONG g_lastReportMilliseconds = 0;

bool ReadOsCursor(Pixels& pixels, bool& inside) noexcept {
    const HWND window = GetActiveWindow();
    POINT point{};
    RECT client{};
    if (window == nullptr || !GetCursorPos(&point) ||
        !ScreenToClient(window, &point) || !GetClientRect(window, &client)) {
        return false;
    }
    pixels = {point.x, point.y};
    inside = point.x >= client.left && point.x < client.right &&
             point.y >= client.top && point.y < client.bottom;
    return true;
}

void __fastcall SendMousePositionHook(std::byte* const imGui,
                                      const Pixels* const position,
                                      const Pixels* const relative) noexcept {
    g_originalSendMousePosition(imGui, position, relative);
    if (!g_enabled || g_applying || imGui == nullptr || position == nullptr ||
        GetCurrentThreadId() != g_gameThread) {
        return;
    }
    g_fedImGui = imGui;
    g_fedTick = g_tick;
    g_fedPixels = *position;
    ++g_diagnostics.feeds;

    // Check that the OS cursor is the same quantity the game is feeding.
    Pixels os{};
    bool inside = false;
    if (g_getRelativeMouseMode() == 0 && ReadOsCursor(os, inside) && inside) {
        const int difference = std::max(std::abs(os.x - position->x),
                                        std::abs(os.y - position->y));
        g_diagnostics.largestTickDifference =
            std::max(g_diagnostics.largestTickDifference, difference);
        if (difference <= kAgreementPixels) {
            g_hasAgreement = true;
            ++g_diagnostics.agreeingTicks;
        }
    }
}

template <typename T>
T ReadAt(const std::byte* const base, const std::size_t offset) noexcept {
    T result{};
    std::memcpy(&result, base + offset, sizeof(result));
    return result;
}

bool SameBits(const float left, const float right) noexcept {
    return std::memcmp(&left, &right, sizeof(float)) == 0;
}

// The GUI is built once per fixed tick into a list of draw commands that is
// then drawn every presented frame. Commands placed exactly at the mouse
// position (the cursor, and anything a script pins to it) are remembered so
// they can be moved with the refreshed position between ticks.
void __fastcall DrawImageHook(std::byte* const imGui, void* const graphic,
                              float* const position, float* const size,
                              void* const a, void* const b, void* const c,
                              void* const d, void* const e) noexcept {
    bool anchored = false;
    float anchorX = 0.0F;
    float anchorY = 0.0F;
    if (g_enabled && imGui != nullptr && position != nullptr &&
        imGui == g_fedImGui && GetCurrentThreadId() == g_gameThread) {
        for (const std::size_t offset :
             {kImGuiMousePositionOffset, kImGuiPreviousPositionOffset}) {
            const float x = ReadAt<float>(imGui, offset);
            const float y = ReadAt<float>(imGui, offset + sizeof(float));
            if (SameBits(position[0], x) && SameBits(position[1], y)) {
                anchored = true;
                anchorX = x;
                anchorY = y;
                break;
            }
        }
    }
    g_originalDrawImage(imGui, graphic, position, size, a, b, c, d, e);
    if (!g_enabled || imGui == nullptr || imGui != g_fedImGui ||
        GetCurrentThreadId() != g_gameThread) {
        return;
    }
    std::byte* const begin = ReadAt<std::byte*>(imGui, kImGuiCommandsBeginOffset);
    std::byte* const end = ReadAt<std::byte*>(imGui, kImGuiCommandsEndOffset);
    if (begin == nullptr || end <= begin ||
        static_cast<std::size_t>(end - begin) % kCommandSize != 0U) {
        return;
    }
    const std::size_t index =
        static_cast<std::size_t>(end - begin) / kCommandSize - 1U;
    // The list only grows within a build, so an index that did not advance
    // means the game cleared it and a new build has begun. Entries from the
    // old build must go even if this build is longer than the last.
    if (g_lastImageIndexValid && index <= g_lastImageIndex &&
        g_anchoredCount != 0U) {
        g_anchoredCount = 0;
        ++g_diagnostics.buildRestarts;
    }
    g_lastImageIndex = index;
    g_lastImageIndexValid = true;
    if (!anchored) {
        return;
    }
    // A lower or equal index than the last one means a new build started.
    if (g_anchoredCount != 0U &&
        (g_anchoredImGui != imGui ||
         index <= g_anchored[g_anchoredCount - 1U].index)) {
        g_anchoredCount = 0;
    }
    if (g_anchoredCount >= g_anchored.size()) {
        return;
    }
    if (g_anchoredCount == 0U) {
        ++g_buildSerial;
    }
    const std::byte* const command = begin + index * kCommandSize;
    const float storedX = ReadAt<float>(command, kCommandPositionOffset);
    const float storedY =
        ReadAt<float>(command, kCommandPositionOffset + sizeof(float));
    // The newest command must be this image: its position is the graphic's
    // own offset plus the position that was passed in.
    const auto* const graphicBytes = static_cast<const std::byte*>(graphic);
    if (graphicBytes == nullptr ||
        !SameBits(storedX, ReadAt<float>(graphicBytes, kGraphicOffsetOffset) +
                               anchorX) ||
        !SameBits(storedY,
                  ReadAt<float>(graphicBytes,
                                kGraphicOffsetOffset + sizeof(float)) +
                      anchorY)) {
        ++g_diagnostics.anchorMismatches;
        return;
    }
    AnchoredCommand& entry = g_anchored[g_anchoredCount++];
    entry.index = index;
    entry.baseX = storedX;
    entry.baseY = storedY;
    entry.writtenX = entry.baseX;
    entry.writtenY = entry.baseY;
    entry.anchorX = anchorX;
    entry.anchorY = anchorY;
    g_anchoredImGui = imGui;
    ++g_diagnostics.anchoredCommands;
}

void MoveAnchoredCommands(std::byte* const imGui) noexcept {
    if (g_anchoredCount == 0U || g_anchoredImGui != imGui) {
        return;
    }
    std::byte* const begin = ReadAt<std::byte*>(imGui, kImGuiCommandsBeginOffset);
    std::byte* const end = ReadAt<std::byte*>(imGui, kImGuiCommandsEndOffset);
    if (begin == nullptr || end <= begin) {
        g_anchoredCount = 0;
        return;
    }
    const std::size_t count =
        static_cast<std::size_t>(end - begin) / kCommandSize;
    const float freshX = ReadAt<float>(imGui, kImGuiMousePositionOffset);
    const float freshY =
        ReadAt<float>(imGui, kImGuiMousePositionOffset + sizeof(float));
    for (std::size_t slot = 0; slot < g_anchoredCount; ++slot) {
        AnchoredCommand& entry = g_anchored[slot];
        if (entry.index >= count) {
            g_anchoredCount = 0;
            ++g_diagnostics.anchorMismatches;
            return;
        }
        std::byte* const command = begin + entry.index * kCommandSize;
        // Only touch a command that still holds what was last seen there.
        if (!SameBits(ReadAt<float>(command, kCommandPositionOffset),
                      entry.writtenX) ||
            !SameBits(ReadAt<float>(command,
                                    kCommandPositionOffset + sizeof(float)),
                      entry.writtenY)) {
            g_anchoredCount = 0;
            ++g_diagnostics.anchorMismatches;
            return;
        }
        entry.writtenX = entry.baseX + (freshX - entry.anchorX);
        entry.writtenY = entry.baseY + (freshY - entry.anchorY);
        std::memcpy(command + kCommandPositionOffset, &entry.writtenX,
                    sizeof(float));
        std::memcpy(command + kCommandPositionOffset + sizeof(float),
                    &entry.writtenY, sizeof(float));
        ++g_diagnostics.commandMoves;
    }
}

// Things a script draws at an offset from the mouse (a dragged inventory
// icon) cannot be recognized from one build. They are recognized from two:
// a command that moved by exactly the mouse's movement between consecutive
// builds is following the mouse, and is then carried between ticks like the
// cursor. It stops being carried the moment that no longer holds.
void SnapshotBuild(std::byte* const imGui, const CommandList& list,
                   FollowState& state) noexcept {
    std::swap(state.previous, state.current);
    BuildSnapshot& current = *state.current;
    const BuildSnapshot& previous = *state.previous;
    current.valid = false;
    std::byte* const begin = ReadAt<std::byte*>(imGui, list.beginOffset);
    std::byte* const end = ReadAt<std::byte*>(imGui, list.endOffset);
    if (begin == nullptr || end <= begin || g_anchoredCount == 0U) {
        state.following.fill(false);
        return;
    }
    const std::size_t count =
        static_cast<std::size_t>(end - begin) / list.size;
    if (count > kMaximumCommands) {
        state.following.fill(false);
        return;
    }
    current.valid = true;
    current.imGui = imGui;
    current.count = count;
    current.mouseX = g_anchored[0].anchorX;
    current.mouseY = g_anchored[0].anchorY;
    for (std::size_t index = 0; index < count; ++index) {
        const std::byte* const command = begin + index * list.size;
        current.x[index] = ReadAt<float>(command, list.positionOffset);
        current.y[index] =
            ReadAt<float>(command, list.positionOffset + sizeof(float));
    }
    if (!previous.valid || previous.imGui != imGui ||
        previous.count != count) {
        state.following.fill(false);
        return;
    }
    constexpr float tolerance = 0.05F;
    const float mouseDx = current.mouseX - previous.mouseX;
    const float mouseDy = current.mouseY - previous.mouseY;
    const bool mouseMoved = std::abs(mouseDx) + std::abs(mouseDy) > 0.5F;
    for (std::size_t index = 0; index < count; ++index) {
        bool anchored = false;
        for (std::size_t slot = 0; slot < g_anchoredCount; ++slot) {
            anchored = anchored || (list.holdsCursor &&
                                    g_anchored[slot].index == index);
        }
        const float dx = current.x[index] - previous.x[index];
        const float dy = current.y[index] - previous.y[index];
        bool following = false;
        if (anchored) {
            following = false;
        } else if (mouseMoved) {
            following = std::abs(dx - mouseDx) < tolerance &&
                        std::abs(dy - mouseDy) < tolerance;
        } else {
            following = state.following[index] && std::abs(dx) < tolerance &&
                        std::abs(dy) < tolerance;
        }
        state.following[index] = following;
        state.writtenX[index] = current.x[index];
        state.writtenY[index] = current.y[index];
        if (following) {
            ++g_diagnostics.followingCommands;
        }
    }
}

void MoveFollowingCommands(std::byte* const imGui, const CommandList& list,
                           FollowState& state) noexcept {
    const BuildSnapshot& current = *state.current;
    if (!current.valid || current.imGui != imGui) {
        return;
    }
    std::byte* const begin = ReadAt<std::byte*>(imGui, list.beginOffset);
    std::byte* const end = ReadAt<std::byte*>(imGui, list.endOffset);
    if (begin == nullptr || end <= begin ||
        static_cast<std::size_t>(end - begin) / list.size !=
            current.count) {
        return;
    }
    const float moveX =
        ReadAt<float>(imGui, kImGuiMousePositionOffset) - current.mouseX;
    const float moveY =
        ReadAt<float>(imGui, kImGuiMousePositionOffset + sizeof(float)) -
        current.mouseY;
    for (std::size_t index = 0; index < current.count; ++index) {
        if (!state.following[index]) {
            continue;
        }
        std::byte* const command = begin + index * list.size;
        if (!SameBits(ReadAt<float>(command, list.positionOffset),
                      state.writtenX[index]) ||
            !SameBits(ReadAt<float>(command,
                                    list.positionOffset + sizeof(float)),
                      state.writtenY[index])) {
            state.following[index] = false;
            ++g_diagnostics.anchorMismatches;
            continue;
        }
        state.writtenX[index] = current.x[index] + moveX;
        state.writtenY[index] = current.y[index] + moveY;
        std::memcpy(command + list.positionOffset,
                    &state.writtenX[index], sizeof(float));
        std::memcpy(command + list.positionOffset + sizeof(float),
                    &state.writtenY[index], sizeof(float));
        ++g_diagnostics.followMoves;
    }
}

// The cursor graphic is drawn at the *previous* position field, which the
// per-frame refresh deliberately leaves alone. For the duration of the draw
// only, show it at the refreshed current position instead.
void __fastcall DoMouseHook(std::byte* const imGui, void* const graphic,
                            float* const offset, void* const size) noexcept {
    ++g_diagnostics.cursorDraws;
    const bool substitute = g_enabled && imGui != nullptr &&
        imGui == g_refreshedImGui && g_refreshedTick + 1U >= g_tick &&
        GetCurrentThreadId() == g_gameThread;
    if (!substitute) {
        g_originalDoMouse(imGui, graphic, offset, size);
        return;
    }
    std::array<std::byte, 8> previous{};
    std::memcpy(previous.data(), imGui + kImGuiPreviousPositionOffset,
                previous.size());
    std::memcpy(imGui + kImGuiPreviousPositionOffset,
                imGui + kImGuiMousePositionOffset, previous.size());
    g_originalDoMouse(imGui, graphic, offset, size);
    std::memcpy(imGui + kImGuiPreviousPositionOffset, previous.data(),
                previous.size());
    ++g_diagnostics.cursorDrawsRefreshed;
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
    if (d.feeds != 0U || d.applied != 0U) {
        log::Write(
            log::Level::info,
            L"GUI cursor preview: ticks/position-feeds/agreeing-ticks=" +
                std::to_wstring(d.ticks) + L"/" + std::to_wstring(d.feeds) +
                L"/" + std::to_wstring(d.agreeingTicks) +
                L", frames/applied=" + std::to_wstring(d.frames) + L"/" +
                std::to_wstring(d.applied) +
                L", skipped relative-mode/no-feed/untrusted/no-cursor/" +
                L"outside=" + std::to_wstring(d.relativeModeFrames) + L"/" +
                std::to_wstring(d.noFeedFrames) + L"/" +
                std::to_wstring(d.untrustedFrames) + L"/" +
                std::to_wstring(d.noCursorFrames) + L"/" +
                std::to_wstring(d.outsideFrames) +
                L", largest OS-vs-game difference at a tick=" +
                std::to_wstring(d.largestTickDifference) +
                L" px, largest step applied in a frame=" +
                std::to_wstring(d.largestFrameStep) +
                L" px, cursor draws total/at-refreshed-position=" +
                std::to_wstring(d.cursorDraws) + L"/" +
                std::to_wstring(d.cursorDrawsRefreshed) +
                L", mouse-anchored commands built/moved/mismatched=" +
                std::to_wstring(d.anchoredCommands) + L"/" +
                std::to_wstring(d.commandMoves) + L"/" +
                std::to_wstring(d.anchorMismatches) +
                L", builds detected by list restart=" +
                std::to_wstring(d.buildRestarts) +
                L", mouse-following commands found/moved=" +
                std::to_wstring(d.followingCommands) + L"/" +
                std::to_wstring(d.followMoves) + L".");
    }
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
        error = L"Host module is null while installing the GUI cursor preview.";
        return false;
    }
    auto* const base = reinterpret_cast<std::byte*>(executable);
    std::byte* const target = base + build::Rva(Function::sendMousePosition);
    if (!build::EntryMatches(executable, Function::sendMousePosition)) {
        error = L"GUI mouse-position entry bytes do not match the supported "
                L"build.";
        return false;
    }
    const iat::LocateResult relativeMode =
        iat::Locate(executable, "SDL2.dll", "SDL_GetRelativeMouseMode");
    if (relativeMode.current == nullptr) {
        error = L"Could not locate SDL_GetRelativeMouseMode.";
        return false;
    }
    MH_STATUS status = MH_Initialize();
    if (status != MH_OK && status != MH_ERROR_ALREADY_INITIALIZED) {
        error = MinHookError(L"MH_Initialize", status);
        return false;
    }
    status = MH_CreateHook(
        target, reinterpret_cast<void*>(&SendMousePositionHook),
        reinterpret_cast<void**>(&g_originalSendMousePosition));
    if (status != MH_OK) {
        error = MinHookError(L"MH_CreateHook(GUI mouse position)", status);
        return false;
    }
    g_executableBase = base;
    g_diagnosticsEnabled = diagnosticsEnabled;
    g_getRelativeMouseMode =
        reinterpret_cast<GetRelativeMouseMode>(relativeMode.current);
    status = MH_EnableHook(target);
    if (status != MH_OK) {
        static_cast<void>(MH_RemoveHook(target));
        g_originalSendMousePosition = nullptr;
        g_executableBase = nullptr;
        error = MinHookError(L"MH_EnableHook(GUI mouse position)", status);
        return false;
    }
    // Optional second detour; without it the refreshed position still
    // reaches scripts but the cursor graphic keeps its stock position.
    std::byte* const doMouseTarget = base + build::Rva(Function::doMouse);
    if (build::EntryMatches(executable, Function::doMouse) &&
        MH_CreateHook(doMouseTarget, reinterpret_cast<void*>(&DoMouseHook),
                      reinterpret_cast<void**>(&g_originalDoMouse)) == MH_OK) {
        if (MH_EnableHook(doMouseTarget) != MH_OK) {
            static_cast<void>(MH_RemoveHook(doMouseTarget));
            g_originalDoMouse = nullptr;
        }
    }
    if (g_originalDoMouse == nullptr) {
        log::Write(log::Level::warning,
                   L"GUI cursor preview: cursor-draw detour unavailable.");
    }
    std::byte* const drawImageTarget = base + build::Rva(Function::drawImage);
    if (build::EntryMatches(executable, Function::drawImage) &&
        MH_CreateHook(drawImageTarget,
                      reinterpret_cast<void*>(&DrawImageHook),
                      reinterpret_cast<void**>(&g_originalDrawImage)) ==
            MH_OK) {
        if (MH_EnableHook(drawImageTarget) != MH_OK) {
            static_cast<void>(MH_RemoveHook(drawImageTarget));
            g_originalDrawImage = nullptr;
        }
    }
    if (g_originalDrawImage == nullptr) {
        log::Write(log::Level::warning,
                   L"GUI cursor preview: image-draw detour unavailable.");
    }
    g_enabled = true;
    return true;
}

void Stop() noexcept {
    g_enabled = false;
    if (g_executableBase != nullptr && g_originalSendMousePosition != nullptr) {
        std::byte* const target =
            g_executableBase + build::Rva(Function::sendMousePosition);
        static_cast<void>(MH_DisableHook(target));
        static_cast<void>(MH_RemoveHook(target));
    }
    if (g_executableBase != nullptr && g_originalDoMouse != nullptr) {
        std::byte* const doMouseTarget =
            g_executableBase + build::Rva(Function::doMouse);
        static_cast<void>(MH_DisableHook(doMouseTarget));
        static_cast<void>(MH_RemoveHook(doMouseTarget));
    }
    if (g_executableBase != nullptr && g_originalDrawImage != nullptr) {
        std::byte* const drawImageTarget =
            g_executableBase + build::Rva(Function::drawImage);
        static_cast<void>(MH_DisableHook(drawImageTarget));
        static_cast<void>(MH_RemoveHook(drawImageTarget));
    }
    g_originalDrawImage = nullptr;
    g_anchoredCount = 0;
    g_anchoredImGui = nullptr;
    g_originalDoMouse = nullptr;
    g_refreshedImGui = nullptr;
    g_originalSendMousePosition = nullptr;
    g_getRelativeMouseMode = nullptr;
    g_executableBase = nullptr;
    g_fedImGui = nullptr;
    g_hasAgreement = false;
}

void OnTickCompleted() noexcept {
    if (!g_enabled) {
        return;
    }
    if (g_gameThread == 0) {
        g_gameThread = GetCurrentThreadId();
    }
    ++g_tick;
    ++g_diagnostics.ticks;
    MaybeReport();
}

void OnFrame() noexcept {
    if (!g_enabled || g_gameThread == 0 ||
        GetCurrentThreadId() != g_gameThread ||
        g_originalSendMousePosition == nullptr) {
        return;
    }
    ++g_diagnostics.frames;
    // Only the GUI that was fed on the latest tick is live.
    if (g_fedImGui == nullptr || g_fedTick + 1U < g_tick) {
        ++g_diagnostics.noFeedFrames;
        return;
    }
    if (g_getRelativeMouseMode() != 0) {
        // A captured mouse pins the OS cursor; it says nothing about the
        // game's software cursor.
        ++g_diagnostics.relativeModeFrames;
        return;
    }
    if (!g_hasAgreement) {
        ++g_diagnostics.untrustedFrames;
        return;
    }
    Pixels os{};
    bool inside = false;
    if (!ReadOsCursor(os, inside)) {
        ++g_diagnostics.noCursorFrames;
        return;
    }
    if (!inside) {
        ++g_diagnostics.outsideFrames;
        return;
    }

    // Reuse the game's own pixel-to-GUI conversion, then put back everything
    // it changes except the position: previous position, relative motion,
    // and the first-update flag stay as the last tick left them.
    std::array<std::byte, kPreservedBytes> preserved{};
    std::memcpy(preserved.data(), g_fedImGui + kImGuiPreviousPositionOffset,
                preserved.size());
    const std::byte flag = g_fedImGui[kImGuiFirstUpdateFlagOffset];
    const Pixels noMotion{};
    g_applying = true;
    g_originalSendMousePosition(g_fedImGui, &os, &noMotion);
    g_applying = false;
    std::memcpy(g_fedImGui + kImGuiPreviousPositionOffset, preserved.data(),
                preserved.size());
    g_fedImGui[kImGuiFirstUpdateFlagOffset] = flag;

    g_refreshedImGui = g_fedImGui;
    g_refreshedTick = g_tick;
    // A new build is read before anything in it is moved.
    if (g_snapshotSerial != g_buildSerial) {
        g_snapshotSerial = g_buildSerial;
        SnapshotBuild(g_fedImGui, kImageList, g_imageFollow);
        SnapshotBuild(g_fedImGui, kTextList, g_textFollow);
    }
    MoveAnchoredCommands(g_fedImGui);
    MoveFollowingCommands(g_fedImGui, kImageList, g_imageFollow);
    MoveFollowingCommands(g_fedImGui, kTextList, g_textFollow);
    ++g_diagnostics.applied;
    g_diagnostics.largestFrameStep =
        std::max({g_diagnostics.largestFrameStep,
                  std::abs(os.x - g_fedPixels.x),
                  std::abs(os.y - g_fedPixels.y)});
    g_fedPixels = os;
    if (!g_loggedActive) {
        g_loggedActive = true;
        log::Write(log::Level::info,
                   L"GUI cursor preview active: the visible-cursor position "
                   L"is refreshed every presented frame.");
    }
}

} // namespace arb::gui_cursor
