#pragma once

#include <Windows.h>

#include <string>

namespace arb::gui_cursor {

[[nodiscard]] bool Start(HMODULE executable, bool diagnosticsEnabled,
                         std::wstring& error) noexcept;
void Stop() noexcept;
void OnTickCompleted() noexcept;
// Called on the game thread once per presented frame, before the GUI for
// that frame is built.
void OnFrame() noexcept;

} // namespace arb::gui_cursor
