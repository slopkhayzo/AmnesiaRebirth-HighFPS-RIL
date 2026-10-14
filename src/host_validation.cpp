#include "host_validation.h"

#include <Windows.h>

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <cwchar>
#include <vector>

namespace arb::host {
namespace {

constexpr std::uint32_t kTestedTimestamp = 0x614063E2U;
constexpr std::uint32_t kTestedImageSize = 0x009FD000U;

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

const wchar_t* BaseName(const std::wstring& path) noexcept {
    const std::size_t slash = path.find_last_of(L"\\/");
    return path.c_str() + (slash == std::wstring::npos ? 0 : slash + 1);
}

} // namespace

ValidationResult Validate(const HMODULE executable) {
    ValidationResult result{};
    if (executable == nullptr) {
        result.error = L"Host module handle is null.";
        return result;
    }

    result.path = ModulePath(executable);
    if (result.path.empty()) {
        result.error = L"Could not resolve the host executable path.";
        return result;
    }
    if (_wcsicmp(BaseName(result.path), L"AmnesiaRebirth.exe") != 0) {
        result.error = L"Host executable name is not AmnesiaRebirth.exe: " +
                       result.path;
        return result;
    }

    const auto* base = reinterpret_cast<const std::byte*>(executable);
    const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    if (dos->e_magic != IMAGE_DOS_SIGNATURE || dos->e_lfanew <= 0 ||
        dos->e_lfanew > 1024 * 1024) {
        result.error = L"Mapped host has an invalid DOS header.";
        return result;
    }

    const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(base + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE) {
        result.error = L"Mapped host has an invalid NT signature.";
        return result;
    }
    result.machine = nt->FileHeader.Machine;
    result.timestamp = nt->FileHeader.TimeDateStamp;
    result.imageSize = nt->OptionalHeader.SizeOfImage;
    if (result.machine != IMAGE_FILE_MACHINE_AMD64 ||
        nt->OptionalHeader.Magic != IMAGE_NT_OPTIONAL_HDR64_MAGIC) {
        result.error = L"Mapped host is not an x64 PE32+ image.";
        return result;
    }
    if (result.imageSize < nt->OptionalHeader.SizeOfHeaders ||
        result.imageSize > 0x80000000U) {
        result.error = L"Mapped host has an invalid image size.";
        return result;
    }
    // The build identity is informational only. Compatibility is decided by
    // each feature checking the entry bytes of the game functions it uses,
    // so a different build with the same code is accepted and one with
    // different code is refused feature by feature.
    result.testedBuild = result.timestamp == kTestedTimestamp &&
                         result.imageSize == kTestedImageSize;
    result.compatible = true;
    return result;
}

bool EntryMatches(const HMODULE executable, const std::uintptr_t rva,
                  const std::uint8_t* const signature,
                  const std::size_t size) noexcept {
    if (executable == nullptr || signature == nullptr || size == 0U) {
        return false;
    }
    const auto* const base = reinterpret_cast<const std::byte*>(executable);
    const auto* const dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    if (dos->e_magic != IMAGE_DOS_SIGNATURE || dos->e_lfanew <= 0 ||
        dos->e_lfanew > 1024 * 1024) {
        return false;
    }
    const auto* const nt =
        reinterpret_cast<const IMAGE_NT_HEADERS64*>(base + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE) {
        return false;
    }
    const std::uintptr_t imageSize = nt->OptionalHeader.SizeOfImage;
    if (rva >= imageSize || size > imageSize - rva) {
        return false;
    }
    MEMORY_BASIC_INFORMATION memory{};
    const std::byte* const address = base + rva;
    if (VirtualQuery(address, &memory, sizeof(memory)) != sizeof(memory) ||
        memory.State != MEM_COMMIT ||
        (memory.Protect & (PAGE_NOACCESS | PAGE_GUARD)) != 0U) {
        return false;
    }
    const auto regionEnd =
        reinterpret_cast<std::uintptr_t>(memory.BaseAddress) +
        memory.RegionSize;
    if (size > regionEnd - reinterpret_cast<std::uintptr_t>(address)) {
        return false;
    }
    return std::memcmp(address, signature, size) == 0;
}

bool AddressMatches(const void* const address,
                    const std::uint8_t* const signature,
                    const std::size_t size) noexcept {
    const HMODULE executable = GetModuleHandleW(nullptr);
    const auto base = reinterpret_cast<std::uintptr_t>(executable);
    const auto target = reinterpret_cast<std::uintptr_t>(address);
    return executable != nullptr && target >= base &&
           EntryMatches(executable, target - base, signature, size);
}

} // namespace arb::host

