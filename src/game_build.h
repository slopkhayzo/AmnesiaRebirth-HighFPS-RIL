#pragma once

#include <Windows.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace arb::build {

// Game functions the fix hooks or calls. Their addresses and entry bytes
// differ between builds of AmnesiaRebirth.exe.
enum class Function : std::size_t {
    engineRun,
    updaterRunMessage,
    viewportRender,
    cameraSetPosition,
    cameraSetRotationMatrix,
    currentMicroseconds,
    lookAmount,
    sendMousePosition,
    doMouse,
    drawImage,
    particleUpdateMotion,
    particleUpdateGraphics,
    entitySetMatrix,
    entitySetVarBool,
    entityGetMatrix,
    entityVirtualSetMatrix,
    matrixInverse,
    matrixMultiply,
    nodeMatrixChanged,
    boneSetMatrix,
    entityNameResolver,
    newtonTransformCallback,
    luxPropOnUpdate,
    luxAgentOnUpdate,
    luxAgentGetMesh,
    count,
};

inline constexpr std::size_t kFunctionCount =
    static_cast<std::size_t>(Function::count);
inline constexpr std::size_t kMaximumSignatureSize = 24U;

struct FunctionEntry {
    std::uintptr_t rva = 0;
    std::size_t signatureSize = 0;
    std::array<std::uint8_t, kMaximumSignatureSize> signature{};
};

struct Build {
    const wchar_t* name = nullptr;
    // Identity of the executable the mapping was made from. Informational:
    // a build is recognised by its code, not by these.
    std::uint32_t timestamp = 0;
    std::uint32_t imageSize = 0;
    std::array<FunctionEntry, kFunctionCount> functions{};
    // The global pointer to the game object (input handler owner).
    std::uintptr_t gameGlobalRva = 0;
};

// Every build with a complete mapping.
[[nodiscard]] std::span<const Build> KnownBuilds() noexcept;
// Selects the known build whose cEngine::Run entry bytes match at that
// build's own address, and keeps it for the lookups below. Returns nullptr
// when none matches; every feature then refuses to start.
const Build* Select(HMODULE executable) noexcept;
[[nodiscard]] const Build* Current() noexcept;
// RVA of the function in the selected build; 0 when none was selected.
[[nodiscard]] std::uintptr_t Rva(Function function) noexcept;
[[nodiscard]] std::uintptr_t GameGlobalRva() noexcept;
// True when the selected build's entry bytes for the function match the
// mapped image. False when no build was selected.
[[nodiscard]] bool EntryMatches(HMODULE executable, Function function) noexcept;
[[nodiscard]] const wchar_t* Name(Function function) noexcept;

} // namespace arb::build
