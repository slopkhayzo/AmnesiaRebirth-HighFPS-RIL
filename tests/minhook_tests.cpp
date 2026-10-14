#include <MinHook.h>

#include <Windows.h>

#include <iostream>

namespace {

using AddOne = int(__fastcall*)(int value);
AddOne g_original = nullptr;

__declspec(noinline) int __fastcall AddOneTarget(const int value) noexcept {
    return value + 1;
}

int __fastcall AddOneHook(const int value) noexcept {
    return g_original(value) + 99;
}

} // namespace

int main() {
    volatile AddOne callTarget = &AddOneTarget;
    if (callTarget(7) != 8) {
        std::cerr << "unhooked target returned the wrong value\n";
        return 1;
    }
    if (MH_Initialize() != MH_OK) {
        std::cerr << "MH_Initialize failed\n";
        return 2;
    }
    if (MH_CreateHook(reinterpret_cast<void*>(&AddOneTarget),
                      reinterpret_cast<void*>(&AddOneHook),
                      reinterpret_cast<void**>(&g_original)) != MH_OK) {
        std::cerr << "MH_CreateHook failed\n";
        return 3;
    }
    if (MH_EnableHook(reinterpret_cast<void*>(&AddOneTarget)) != MH_OK) {
        std::cerr << "MH_EnableHook failed\n";
        return 4;
    }
    if (callTarget(7) != 107) {
        std::cerr << "detoured target returned the wrong value\n";
        return 5;
    }
    if (MH_DisableHook(reinterpret_cast<void*>(&AddOneTarget)) != MH_OK ||
        callTarget(7) != 8) {
        std::cerr << "target was not restored\n";
        return 6;
    }
    MH_Uninitialize();
    std::cout << "MinHook x64 detour tests passed\n";
    return 0;
}
