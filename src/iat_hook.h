#pragma once

#include <Windows.h>

#include <string>

namespace arb::iat {

struct LocateResult {
    void** slot = nullptr;
    void* current = nullptr;
    std::wstring error;
};

[[nodiscard]] LocateResult Locate(HMODULE module, const char* importedModule,
                                  const char* importedFunction) noexcept;
[[nodiscard]] bool Replace(void** slot, void* expected, void* replacement,
                           std::wstring& error) noexcept;

} // namespace arb::iat
