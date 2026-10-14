#pragma once

#include <Windows.h>

#include <cstdint>
#include <string>

namespace arb::host {

struct ValidationResult {
    bool compatible = false;
    std::wstring path;
    std::wstring error;
    std::uint16_t machine = 0;
    std::uint32_t imageSize = 0;
    std::uint32_t timestamp = 0;
    // True for the exact executable the fix was developed against.
    bool testedBuild = false;
};

[[nodiscard]] ValidationResult Validate(HMODULE executable);
// True when the bytes at the given RVA lie inside the mapped image, are
// readable, and equal the signature. Safe to call on any build.
[[nodiscard]] bool EntryMatches(HMODULE executable, std::uintptr_t rva,
                                const std::uint8_t* signature,
                                std::size_t size) noexcept;
// The same check for an address inside the process's main executable.
[[nodiscard]] bool AddressMatches(const void* address,
                                  const std::uint8_t* signature,
                                  std::size_t size) noexcept;

} // namespace arb::host

