#include "bootstrap.h"

#include <Windows.h>

BOOL WINAPI DllMain(const HINSTANCE instance, const DWORD reason, LPVOID) {
    if (reason != DLL_PROCESS_ATTACH) {
        return TRUE;
    }

    DisableThreadLibraryCalls(instance);
    const HANDLE bootstrap =
        CreateThread(nullptr, 0, arb::BootstrapThread, instance, 0, nullptr);
    if (bootstrap == nullptr) {
        OutputDebugStringW(
            L"[AmnesiaRebirth.FPSFix] Could not create bootstrap thread; "
            L"no hooks were installed.\n");
        return TRUE;
    }
    CloseHandle(bootstrap);
    return TRUE;
}

