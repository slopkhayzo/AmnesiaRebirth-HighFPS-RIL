#include "iat_hook.h"

#include <Windows.h>

#include <iostream>
#include <string>

namespace {

constexpr DWORD kHookedProcessId = 0x51A7C0DEU;

DWORD WINAPI HookedGetCurrentProcessId() noexcept {
    return kHookedProcessId;
}

} // namespace

int main() {
    // Make sure the linker retains the named import that the test will replace.
    const DWORD actualProcessId = GetCurrentProcessId();
    if (actualProcessId == 0 || actualProcessId == kHookedProcessId) {
        std::cerr << "unexpected real process id\n";
        return 1;
    }

    const arb::iat::LocateResult located = arb::iat::Locate(
        GetModuleHandleW(nullptr), "KERNEL32.dll", "GetCurrentProcessId");
    if (located.slot == nullptr || located.current == nullptr) {
        std::wcerr << L"could not locate test import: " << located.error << L'\n';
        return 2;
    }

    std::wstring error;
    if (!arb::iat::Replace(located.slot, located.current,
                           reinterpret_cast<void*>(&HookedGetCurrentProcessId),
                           error)) {
        std::wcerr << L"could not install test hook: " << error << L'\n';
        return 3;
    }
    if (GetCurrentProcessId() != kHookedProcessId) {
        std::cerr << "replacement import was not called\n";
        return 4;
    }

    error.clear();
    if (!arb::iat::Replace(located.slot,
                           reinterpret_cast<void*>(&HookedGetCurrentProcessId),
                           located.current, error)) {
        std::wcerr << L"could not restore test import: " << error << L'\n';
        return 5;
    }
    if (GetCurrentProcessId() != actualProcessId) {
        std::cerr << "restored import returned the wrong process id\n";
        return 6;
    }

    std::cout << "IAT hook tests passed\n";
    return 0;
}
