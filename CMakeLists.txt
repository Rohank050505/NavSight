# Alternative build for IDEs / CI. The primary, verified build is build.ps1
# (MSVC only, no CMake needed). Keep the two source lists in sync.
#
#   cmake -S . -B build/cmake -G "Visual Studio 18 2026" -A x64
#   cmake --build build/cmake --config Release
#   ctest --test-dir build/cmake -C Release
cmake_minimum_required(VERSION 3.21)
project(NavSight VERSION 1.0.0 LANGUAGES CXX)

if(NOT WIN32)
    message(FATAL_ERROR "NavSight currently targets Windows only.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")  # static CRT

set(XPSDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/XPSDK" CACHE PATH "X-Plane SDK root")
if(NOT EXISTS "${XPSDK_DIR}/CHeaders/XPLM/XPLMDefs.h")
    message(FATAL_ERROR "X-Plane SDK not found in ${XPSDK_DIR}. Run '.\\build.ps1 -FetchSdk' first.")
endif()

if(MSVC)
    add_compile_options(/W4 /permissive- /utf-8)
endif()

# X-Plane-independent logic: nav math, config, renderer. Also used by tests.
add_library(navsight_core STATIC
    src/core/Log.cpp
    src/core/NavLog.cpp
    src/core/Config.cpp
    src/nav/InstrumentModel.cpp
    src/nav/AxisGuidance.cpp
    src/nav/ApproachNaming.cpp
    src/nav/NavValidation.cpp
    src/nav/TrajectoryPredictor.cpp
    src/nav/RunwayOverride.cpp
    src/nav/SourceCheck.cpp
    src/render/Palette.cpp
    src/render/RunwayPicker.cpp
    src/render/DisplaySmoother.cpp
    src/render/Canvas.cpp
    src/render/StrokeFont.cpp
    src/render/RenderKit.cpp
    src/render/GuidanceText.cpp
    src/render/DialLayers.cpp
    src/render/PanelLayers.cpp
    src/render/InstrumentRenderer.cpp
)
target_include_directories(navsight_core PUBLIC src)

# The plugin (.xpl is a DLL).
add_library(NavSight SHARED
    src/plugin/PluginMain.cpp
    src/app/NavSightApp.cpp
    src/data/DataRef.cpp
    src/data/ApproachResolver.cpp
    src/data/XPlaneDataSource.cpp
    src/render/GlCanvas.cpp
    src/ui/InstrumentWindow.cpp
)
target_include_directories(NavSight SYSTEM PRIVATE "${XPSDK_DIR}/CHeaders/XPLM")
target_compile_definitions(NavSight PRIVATE
    IBM=1 APL=0 LIN=0 WIN32_LEAN_AND_MEAN NOMINMAX _CRT_SECURE_NO_WARNINGS
    XPLM200=1 XPLM210=1 XPLM300=1 XPLM301=1 XPLM303=1 XPLM400=1 XPLM410=1 XPLM420=1 XPLM430=1
)
target_link_libraries(NavSight PRIVATE navsight_core "${XPSDK_DIR}/Libraries/Win/XPLM_64.lib" opengl32)
set_target_properties(NavSight PROPERTIES
    PREFIX ""
    SUFFIX ".xpl"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIG>/NavSight/win_x64"
)

enable_testing()
add_executable(NavSight_tests tests/TestMain.cpp)
target_link_libraries(NavSight_tests PRIVATE navsight_core)
add_test(NAME NavSight_tests COMMAND NavSight_tests)
