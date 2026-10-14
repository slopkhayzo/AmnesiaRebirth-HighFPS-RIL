#pragma once

#include <string>
#include <string_view>

namespace arb::log {

enum class Level {
    debug,
    info,
    warning,
    error,
};

[[nodiscard]] bool Initialize(const std::wstring& path);
void SetDiagnosticsEnabled(bool enabled) noexcept;
void Write(Level level, std::wstring_view message);

} // namespace arb::log

