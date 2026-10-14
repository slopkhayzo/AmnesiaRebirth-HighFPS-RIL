#pragma once

#include <Windows.h>

#include <string>

namespace arb::camera_hooks {

[[nodiscard]] bool Start(HMODULE executable, std::wstring& error) noexcept;
void Stop() noexcept;

} // namespace arb::camera_hooks
