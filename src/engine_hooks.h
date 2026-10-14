#pragma once

#include <Windows.h>

#include <cstddef>
#include <string>

namespace arb::engine {

[[nodiscard]] bool StartRenderUnlock(HMODULE executable,
                                     std::wstring& error) noexcept;
[[nodiscard]] std::byte* Instance() noexcept;
void StopRenderUnlock() noexcept;

} // namespace arb::engine
