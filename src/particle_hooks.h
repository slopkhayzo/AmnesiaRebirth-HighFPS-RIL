#pragma once

#include "presentation_clock.h"

#include <Windows.h>

#include <string>

namespace arb::particle_hooks {

[[nodiscard]] bool Start(HMODULE executable, bool diagnosticsEnabled,
                         std::wstring& error) noexcept;
void Stop() noexcept;
void OnTickCompleted() noexcept;
// Bracket the listener viewport's render with the sample it is drawn at.
void BeginRender(const clock::Sample& sample) noexcept;
void EndRender() noexcept;

} // namespace arb::particle_hooks
