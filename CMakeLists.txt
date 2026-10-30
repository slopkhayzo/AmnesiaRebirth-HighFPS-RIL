cmake_minimum_required(VERSION 3.24)

project(
    AmnesiaRebirthFpsFix
    VERSION 1.1.0
    DESCRIPTION "High-frame-rate presentation fix for Amnesia: Rebirth"
    LANGUAGES C CXX
)

include(CTest)

if(NOT WIN32 OR NOT MSVC)
    message(FATAL_ERROR "AmnesiaRebirth.FPSFix supports 64-bit Windows/MSVC only")
endif()

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Configure an x64 build environment; AmnesiaRebirth.exe is 64-bit")
endif()

set(AR_COMMON_COMPILE_OPTIONS
    /W4
    /WX
    /permissive-
    /utf-8
    /Zc:preprocessor
)

add_library(minhook STATIC
    third_party/minhook/src/buffer.c
    third_party/minhook/src/hook.c
    third_party/minhook/src/trampoline.c
    third_party/minhook/src/hde/hde64.c
)
target_include_directories(minhook PUBLIC third_party/minhook/include)
target_include_directories(minhook PRIVATE third_party/minhook/src)
target_compile_definitions(minhook PRIVATE _CRT_SECURE_NO_WARNINGS)
set_property(TARGET minhook PROPERTY MSVC_RUNTIME_LIBRARY
    "MultiThreaded$<$<CONFIG:Debug>:Debug>")

add_library(AmnesiaRebirthFpsFix MODULE
    src/bootstrap.cpp
    src/camera_attachment.cpp
    src/camera_hooks.cpp
    src/camera_interpolation.cpp
    src/config.cpp
    src/dllmain.cpp
    src/engine_hooks.cpp
    src/frame_cadence.cpp
    src/frame_limiter.cpp
    src/game_build.cpp
    src/gui_cursor.cpp
    src/host_validation.cpp
    src/iat_hook.cpp
    src/logger.cpp
    src/mouse_hooks.cpp
    src/mouse_preview.cpp
    src/particle_hooks.cpp
    src/pose_interpolation.cpp
    src/presentation_clock.cpp
    src/root_interpolation.cpp
    src/root_probe.cpp
    src/swap_telemetry.cpp
)

target_compile_features(AmnesiaRebirthFpsFix PRIVATE cxx_std_20)
target_compile_definitions(AmnesiaRebirthFpsFix PRIVATE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
    UNICODE
    _UNICODE
)
target_compile_options(AmnesiaRebirthFpsFix PRIVATE ${AR_COMMON_COMPILE_OPTIONS})
target_link_libraries(AmnesiaRebirthFpsFix PRIVATE minhook)
target_link_options(AmnesiaRebirthFpsFix PRIVATE
    /DYNAMICBASE
    /NXCOMPAT
    /GUARD:CF
    /INCREMENTAL:NO
)
set_property(TARGET AmnesiaRebirthFpsFix PROPERTY
    MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
)
set_target_properties(AmnesiaRebirthFpsFix PROPERTIES
    OUTPUT_NAME "AmnesiaRebirth.FPSFix"
    PREFIX ""
    SUFFIX ".asi"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

add_custom_command(TARGET AmnesiaRebirthFpsFix POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${CMAKE_SOURCE_DIR}/config/AmnesiaRebirth.FPSFix.ini"
            "$<TARGET_FILE_DIR:AmnesiaRebirthFpsFix>/AmnesiaRebirth.FPSFix.ini"
    VERBATIM
)

install(TARGETS AmnesiaRebirthFpsFix
    LIBRARY DESTINATION .
    RUNTIME DESTINATION .
)
install(FILES config/AmnesiaRebirth.FPSFix.ini DESTINATION .)

set(AR_PACKAGE_SCRIPT "${CMAKE_SOURCE_DIR}/cmake/package.cmake")
set(AR_ULTIMATE_ASI_LOADER_DLL "" CACHE FILEPATH
    "Optional x64 Ultimate ASI Loader version.dll used for loader-bundled packages")
add_custom_target(package_plugin_only
    COMMAND "${CMAKE_COMMAND}"
            -DMODE=plugin-only
            -DVERSION=${PROJECT_VERSION}
            -DPLUGIN_FILE=$<TARGET_FILE:AmnesiaRebirthFpsFix>
            -DINI_FILE=${CMAKE_SOURCE_DIR}/config/AmnesiaRebirth.FPSFix.ini
            -DREADME_FILE=${CMAKE_SOURCE_DIR}/packaging/README.txt
            -DMINHOOK_LICENSE_FILE=${CMAKE_SOURCE_DIR}/third_party/minhook/LICENSE.txt
            -DOUTPUT_DIR=${CMAKE_SOURCE_DIR}/dist/release/AmnesiaRebirth.FPSFix-${PROJECT_VERSION}-plugin-only
            -DARCHIVE_FILE=${CMAKE_SOURCE_DIR}/dist/AmnesiaRebirth.FPSFix-${PROJECT_VERSION}-plugin-only.zip
            -P "${AR_PACKAGE_SCRIPT}"
    DEPENDS AmnesiaRebirthFpsFix
    VERBATIM
)

if(AR_ULTIMATE_ASI_LOADER_DLL AND EXISTS "${AR_ULTIMATE_ASI_LOADER_DLL}")
    add_custom_target(package_loader_bundled
        COMMAND "${CMAKE_COMMAND}"
                -DMODE=loader-bundled
                -DVERSION=${PROJECT_VERSION}
                -DPLUGIN_FILE=$<TARGET_FILE:AmnesiaRebirthFpsFix>
                -DINI_FILE=${CMAKE_SOURCE_DIR}/config/AmnesiaRebirth.FPSFix.ini
                -DREADME_FILE=${CMAKE_SOURCE_DIR}/packaging/README-loader.txt
                -DMINHOOK_LICENSE_FILE=${CMAKE_SOURCE_DIR}/third_party/minhook/LICENSE.txt
                -DLOADER_FILE=${AR_ULTIMATE_ASI_LOADER_DLL}
                -DLOADER_LICENSE_FILE=${CMAKE_SOURCE_DIR}/third_party/ultimate-asi-loader/LICENSE.txt
                -DOUTPUT_DIR=${CMAKE_SOURCE_DIR}/dist/release/AmnesiaRebirth.FPSFix-${PROJECT_VERSION}-with-Ultimate-ASI-Loader
                -DARCHIVE_FILE=${CMAKE_SOURCE_DIR}/dist/AmnesiaRebirth.FPSFix-${PROJECT_VERSION}-with-Ultimate-ASI-Loader.zip
                -P "${AR_PACKAGE_SCRIPT}"
        DEPENDS AmnesiaRebirthFpsFix
        VERBATIM
    )
endif()

if(BUILD_TESTING)
    add_executable(frame_cadence_tests
        tests/frame_cadence_tests.cpp
        src/frame_cadence.cpp
    )
    target_include_directories(frame_cadence_tests PRIVATE src)
    target_compile_features(frame_cadence_tests PRIVATE cxx_std_20)
    target_compile_options(frame_cadence_tests PRIVATE ${AR_COMMON_COMPILE_OPTIONS})
    set_property(TARGET frame_cadence_tests PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    add_test(NAME frame_cadence COMMAND frame_cadence_tests)

    add_executable(presentation_clock_tests
        tests/presentation_clock_tests.cpp
        src/presentation_clock.cpp
    )
    target_include_directories(presentation_clock_tests PRIVATE src)
    target_compile_features(presentation_clock_tests PRIVATE cxx_std_20)
    target_compile_options(presentation_clock_tests PRIVATE ${AR_COMMON_COMPILE_OPTIONS})
    set_property(TARGET presentation_clock_tests PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    add_test(NAME presentation_clock COMMAND presentation_clock_tests)

    add_executable(camera_interpolation_tests
        tests/camera_interpolation_tests.cpp
        src/camera_interpolation.cpp
        src/root_interpolation.cpp
    )
    target_include_directories(camera_interpolation_tests PRIVATE src)
    target_compile_features(camera_interpolation_tests PRIVATE cxx_std_20)
    target_compile_options(camera_interpolation_tests PRIVATE ${AR_COMMON_COMPILE_OPTIONS})
    set_property(TARGET camera_interpolation_tests PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    add_test(NAME camera_interpolation COMMAND camera_interpolation_tests)

    add_executable(root_interpolation_tests
        tests/root_interpolation_tests.cpp
        src/root_interpolation.cpp
    )
    target_include_directories(root_interpolation_tests PRIVATE src)
    target_compile_features(root_interpolation_tests PRIVATE cxx_std_20)
    target_compile_options(root_interpolation_tests PRIVATE ${AR_COMMON_COMPILE_OPTIONS})
    set_property(TARGET root_interpolation_tests PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    add_test(NAME root_interpolation COMMAND root_interpolation_tests)

    add_executable(pose_interpolation_tests
        tests/pose_interpolation_tests.cpp
        src/pose_interpolation.cpp
        src/root_interpolation.cpp
    )
    target_include_directories(pose_interpolation_tests PRIVATE src)
    target_compile_features(pose_interpolation_tests PRIVATE cxx_std_20)
    target_compile_options(pose_interpolation_tests PRIVATE ${AR_COMMON_COMPILE_OPTIONS})
    set_property(TARGET pose_interpolation_tests PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    add_test(NAME pose_interpolation COMMAND pose_interpolation_tests)

    add_executable(camera_attachment_tests
        tests/camera_attachment_tests.cpp
        src/camera_attachment.cpp
        src/root_interpolation.cpp
    )
    target_include_directories(camera_attachment_tests PRIVATE src)
    target_compile_features(camera_attachment_tests PRIVATE cxx_std_20)
    target_compile_options(camera_attachment_tests PRIVATE ${AR_COMMON_COMPILE_OPTIONS})
    set_property(TARGET camera_attachment_tests PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    add_test(NAME camera_attachment COMMAND camera_attachment_tests)

    add_executable(mouse_preview_tests
        tests/mouse_preview_tests.cpp
        src/mouse_preview.cpp
    )
    target_include_directories(mouse_preview_tests PRIVATE src)
    target_compile_features(mouse_preview_tests PRIVATE cxx_std_20)
    target_compile_options(mouse_preview_tests PRIVATE ${AR_COMMON_COMPILE_OPTIONS})
    set_property(TARGET mouse_preview_tests PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    add_test(NAME mouse_preview COMMAND mouse_preview_tests)

    add_executable(frame_limiter_tests
        tests/frame_limiter_tests.cpp
        src/frame_limiter.cpp
    )
    target_include_directories(frame_limiter_tests PRIVATE src)
    target_compile_features(frame_limiter_tests PRIVATE cxx_std_20)
    target_compile_options(frame_limiter_tests PRIVATE ${AR_COMMON_COMPILE_OPTIONS})
    set_property(TARGET frame_limiter_tests PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    add_test(NAME frame_limiter COMMAND frame_limiter_tests)

    add_executable(iat_hook_tests
        tests/iat_hook_tests.cpp
        src/iat_hook.cpp
    )
    target_include_directories(iat_hook_tests PRIVATE src)
    target_compile_features(iat_hook_tests PRIVATE cxx_std_20)
    target_compile_definitions(iat_hook_tests PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        UNICODE
        _UNICODE
    )
    target_compile_options(iat_hook_tests PRIVATE ${AR_COMMON_COMPILE_OPTIONS})
    target_link_options(iat_hook_tests PRIVATE /GUARD:CF)
    set_property(TARGET iat_hook_tests PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    add_test(NAME iat_hook COMMAND iat_hook_tests)

    set(AR_GOG_EXECUTABLE "" CACHE FILEPATH
        "Optional GOG AmnesiaRebirth.exe checked against the build table")
    set(AR_STEAM_EXECUTABLE "" CACHE FILEPATH
        "Optional Steam AmnesiaRebirth.exe checked against the build table")
    set(AR_GAME_BUILD_TEST_ARGS)
    if(AR_GOG_EXECUTABLE AND EXISTS "${AR_GOG_EXECUTABLE}")
        list(APPEND AR_GAME_BUILD_TEST_ARGS "GOG=${AR_GOG_EXECUTABLE}")
    endif()
    if(AR_STEAM_EXECUTABLE AND EXISTS "${AR_STEAM_EXECUTABLE}")
        list(APPEND AR_GAME_BUILD_TEST_ARGS "Steam=${AR_STEAM_EXECUTABLE}")
    endif()
    add_executable(game_build_tests
        tests/game_build_tests.cpp
        src/game_build.cpp
        src/host_validation.cpp
    )
    target_include_directories(game_build_tests PRIVATE src)
    target_compile_features(game_build_tests PRIVATE cxx_std_20)
    target_compile_definitions(game_build_tests PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        UNICODE
        _UNICODE
    )
    target_compile_options(game_build_tests PRIVATE ${AR_COMMON_COMPILE_OPTIONS})
    set_property(TARGET game_build_tests PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    add_test(NAME game_build COMMAND game_build_tests ${AR_GAME_BUILD_TEST_ARGS})

    add_executable(minhook_tests tests/minhook_tests.cpp)
    target_compile_features(minhook_tests PRIVATE cxx_std_20)
    target_compile_options(minhook_tests PRIVATE ${AR_COMMON_COMPILE_OPTIONS})
    target_link_libraries(minhook_tests PRIVATE minhook)
    target_link_options(minhook_tests PRIVATE /GUARD:CF)
    set_property(TARGET minhook_tests PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    add_test(NAME minhook_x64_detour COMMAND minhook_tests)

    add_executable(asi_bootstrap_host tests/asi_bootstrap_host.cpp)
    target_compile_features(asi_bootstrap_host PRIVATE cxx_std_20)
    target_compile_definitions(asi_bootstrap_host PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
        UNICODE
        _UNICODE
    )
    target_compile_options(asi_bootstrap_host PRIVATE ${AR_COMMON_COMPILE_OPTIONS})
    set_property(TARGET asi_bootstrap_host PROPERTY
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )
    set_target_properties(asi_bootstrap_host PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
    add_test(
        NAME asi_bootstrap_rejects_incompatible_host
        COMMAND asi_bootstrap_host "$<TARGET_FILE:AmnesiaRebirthFpsFix>"
    )
endif()

