#include "game_build.h"

#include <Windows.h>

#include <cstdint>
#include <cwchar>
#include <iostream>
#include <set>
#include <string>

// Checks the build table's invariants. Optional arguments of the form
// <build name>=<path to AmnesiaRebirth.exe> also map that executable as an
// image (no code runs) and require that it selects that build and that every
// function's entry bytes match.

namespace {

int CheckTable() {
    const auto builds = arb::build::KnownBuilds();
    if (builds.empty()) {
        std::cerr << "no known builds\n";
        return 1;
    }
    std::set<std::uintptr_t> engineRuns;
    for (const arb::build::Build& build : builds) {
        if (build.name == nullptr || build.gameGlobalRva == 0U ||
            build.gameGlobalRva >= build.imageSize) {
            std::cerr << "build without a name or valid game global\n";
            return 2;
        }
        std::set<std::uintptr_t> rvas;
        for (std::size_t i = 0; i < arb::build::kFunctionCount; ++i) {
            const arb::build::FunctionEntry& entry = build.functions[i];
            if (entry.rva == 0U || entry.rva >= build.imageSize ||
                entry.signatureSize < 8U ||
                entry.signatureSize > arb::build::kMaximumSignatureSize ||
                !rvas.insert(entry.rva).second) {
                std::wcerr << build.name << L": bad entry for "
                           << arb::build::Name(
                                  static_cast<arb::build::Function>(i))
                           << L'\n';
                return 3;
            }
        }
        // Selection tells builds apart by where cEngine::Run lives.
        if (!engineRuns
                 .insert(build.functions[static_cast<std::size_t>(
                                             arb::build::Function::engineRun)]
                             .rva)
                 .second) {
            std::wcerr << build.name << L": cEngine::Run RVA is not unique\n";
            return 4;
        }
    }
    if (arb::build::Rva(arb::build::Function::engineRun) != 0U ||
        arb::build::EntryMatches(GetModuleHandleW(nullptr),
                                 arb::build::Function::engineRun)) {
        std::cerr << "lookups must fail before a build is selected\n";
        return 5;
    }
    if (arb::build::Select(GetModuleHandleW(nullptr)) != nullptr) {
        std::cerr << "the test executable was taken for a game build\n";
        return 6;
    }
    return 0;
}

int CheckExecutable(const std::wstring& argument) {
    const std::size_t separator = argument.find(L'=');
    if (separator == std::wstring::npos) {
        std::wcerr << L"expected <build>=<path>: " << argument << L'\n';
        return 10;
    }
    const std::wstring expected = argument.substr(0, separator);
    const std::wstring path = argument.substr(separator + 1);
    const HMODULE mapped = LoadLibraryExW(path.c_str(), nullptr,
                                          LOAD_LIBRARY_AS_IMAGE_RESOURCE);
    if (mapped == nullptr) {
        std::wcerr << L"could not map " << path << L": " << GetLastError()
                   << L'\n';
        return 11;
    }
    // The low bits of an image-resource handle are flags.
    const auto image = reinterpret_cast<HMODULE>(
        reinterpret_cast<std::uintptr_t>(mapped) & ~std::uintptr_t{3});
    int result = 0;
    const arb::build::Build* const selected = arb::build::Select(image);
    if (selected == nullptr || expected != selected->name) {
        std::wcerr << path << L": selected "
                   << (selected == nullptr ? L"no build" : selected->name)
                   << L", expected " << expected << L'\n';
        result = 12;
    } else {
        for (std::size_t i = 0; i < arb::build::kFunctionCount; ++i) {
            const auto function = static_cast<arb::build::Function>(i);
            if (!arb::build::EntryMatches(image, function)) {
                std::wcerr << path << L": entry bytes differ for "
                           << arb::build::Name(function) << L'\n';
                result = 13;
            }
        }
    }
    if (result == 0) {
        std::wcout << expected << L" executable matches all "
                   << arb::build::kFunctionCount << L" functions\n";
    }
    FreeLibrary(mapped);
    return result;
}

} // namespace

int wmain(const int argc, wchar_t** const argv) {
    if (const int result = CheckTable(); result != 0) {
        return result;
    }
    for (int i = 1; i < argc; ++i) {
        if (const int result = CheckExecutable(argv[i]); result != 0) {
            return result;
        }
    }
    std::cout << "game build tests passed\n";
    return 0;
}
