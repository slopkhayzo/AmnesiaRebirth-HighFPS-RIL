#include "iat_hook.h"

#include <cstddef>
#include <cstring>

namespace arb::iat {
namespace {

bool ContainsRva(const std::uint32_t rva, const std::size_t bytes,
                 const std::uint32_t imageSize) noexcept {
    return rva < imageSize && bytes <= imageSize - rva;
}

template <typename T>
T* AtRva(std::byte* base, const std::uint32_t rva,
         const std::uint32_t imageSize) noexcept {
    if (!ContainsRva(rva, sizeof(T), imageSize)) {
        return nullptr;
    }
    return reinterpret_cast<T*>(base + rva);
}

bool EqualBoundedName(const char* candidate, const char* expected,
                      const std::size_t maximumBytes) noexcept {
    const std::size_t length = strnlen_s(candidate, maximumBytes);
    return length != maximumBytes && _stricmp(candidate, expected) == 0;
}

} // namespace

LocateResult Locate(const HMODULE module, const char* importedModule,
                    const char* importedFunction) noexcept {
    LocateResult result{};
    if (module == nullptr || importedModule == nullptr ||
        importedFunction == nullptr) {
        result.error = L"Invalid import lookup argument.";
        return result;
    }

    auto* const base = reinterpret_cast<std::byte*>(module);
    const auto* const dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
    if (dos->e_magic != IMAGE_DOS_SIGNATURE || dos->e_lfanew <= 0) {
        result.error = L"Invalid DOS header while locating import.";
        return result;
    }
    const auto* const nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(
        base + static_cast<std::size_t>(dos->e_lfanew));
    if (nt->Signature != IMAGE_NT_SIGNATURE ||
        nt->OptionalHeader.Magic != IMAGE_NT_OPTIONAL_HDR64_MAGIC) {
        result.error = L"Invalid PE64 header while locating import.";
        return result;
    }

    const std::uint32_t imageSize = nt->OptionalHeader.SizeOfImage;
    const IMAGE_DATA_DIRECTORY imports =
        nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
    if (imports.VirtualAddress == 0 || imports.Size < sizeof(IMAGE_IMPORT_DESCRIPTOR) ||
        !ContainsRva(imports.VirtualAddress, imports.Size, imageSize)) {
        result.error = L"Host has no valid import directory.";
        return result;
    }

    const std::size_t descriptorCount =
        imports.Size / sizeof(IMAGE_IMPORT_DESCRIPTOR);
    auto* const descriptors = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR*>(
        base + imports.VirtualAddress);
    for (std::size_t descriptorIndex = 0; descriptorIndex < descriptorCount;
         ++descriptorIndex) {
        const IMAGE_IMPORT_DESCRIPTOR& descriptor = descriptors[descriptorIndex];
        if (descriptor.Name == 0 && descriptor.FirstThunk == 0) {
            break;
        }
        if (!ContainsRva(descriptor.Name, 1, imageSize)) {
            continue;
        }
        const char* const moduleName =
            reinterpret_cast<const char*>(base + descriptor.Name);
        if (!EqualBoundedName(moduleName, importedModule,
                              imageSize - descriptor.Name)) {
            continue;
        }
        if (descriptor.OriginalFirstThunk == 0 || descriptor.FirstThunk == 0) {
            result.error = L"Named import lookup table is unavailable.";
            return result;
        }

        for (std::size_t thunkIndex = 0;; ++thunkIndex) {
            const std::uint64_t lookupRva64 =
                static_cast<std::uint64_t>(descriptor.OriginalFirstThunk) +
                thunkIndex * sizeof(IMAGE_THUNK_DATA64);
            const std::uint64_t addressRva64 =
                static_cast<std::uint64_t>(descriptor.FirstThunk) +
                thunkIndex * sizeof(IMAGE_THUNK_DATA64);
            if (lookupRva64 > UINT32_MAX || addressRva64 > UINT32_MAX) {
                break;
            }
            auto* const lookup = AtRva<IMAGE_THUNK_DATA64>(
                base, static_cast<std::uint32_t>(lookupRva64), imageSize);
            auto* const address = AtRva<IMAGE_THUNK_DATA64>(
                base, static_cast<std::uint32_t>(addressRva64), imageSize);
            if (lookup == nullptr || address == nullptr ||
                lookup->u1.AddressOfData == 0) {
                break;
            }
            if (IMAGE_SNAP_BY_ORDINAL64(lookup->u1.Ordinal)) {
                continue;
            }
            if (lookup->u1.AddressOfData > UINT32_MAX) {
                continue;
            }
            const std::uint32_t nameRva =
                static_cast<std::uint32_t>(lookup->u1.AddressOfData);
            auto* const import = AtRva<IMAGE_IMPORT_BY_NAME>(base, nameRva, imageSize);
            if (import == nullptr) {
                continue;
            }
            const std::uint32_t functionNameRva =
                nameRva + static_cast<std::uint32_t>(offsetof(IMAGE_IMPORT_BY_NAME, Name));
            if (!ContainsRva(functionNameRva, 1, imageSize)) {
                continue;
            }
            const char* const functionName =
                reinterpret_cast<const char*>(base + functionNameRva);
            if (EqualBoundedName(functionName, importedFunction,
                                 imageSize - functionNameRva)) {
                result.slot = reinterpret_cast<void**>(&address->u1.Function);
                result.current = *result.slot;
                return result;
            }
        }
        result.error = L"Function was not present in the requested import module.";
        return result;
    }

    result.error = L"Requested import module was not present in the host.";
    return result;
}

bool Replace(void** const slot, void* const expected, void* const replacement,
             std::wstring& error) noexcept {
    if (slot == nullptr || expected == nullptr || replacement == nullptr) {
        error = L"Invalid import replacement argument.";
        return false;
    }
    DWORD previousProtection = 0;
    if (!VirtualProtect(slot, sizeof(*slot), PAGE_READWRITE, &previousProtection)) {
        error = L"VirtualProtect could not make the import slot writable (error " +
                std::to_wstring(GetLastError()) + L").";
        return false;
    }
    void* const observed = InterlockedCompareExchangePointer(
        slot, replacement, expected);
    DWORD ignored = 0;
    const BOOL restored =
        VirtualProtect(slot, sizeof(*slot), previousProtection, &ignored);
    FlushInstructionCache(GetCurrentProcess(), slot, sizeof(*slot));
    if (observed != expected) {
        error = L"Import slot changed before hook installation.";
        return false;
    }
    if (!restored) {
        // The exchange already succeeded. Reporting failure here would make a
        // caller assume that no hook exists and could invalidate its trampoline.
        error = L"Hook installed, but import-slot protection restoration failed.";
    }
    return true;
}

} // namespace arb::iat
