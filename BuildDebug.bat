@echo off
REM Build MAD in Debug configuration.
if not exist build_debug (
  mkdir build_debug
)
conan install . --output-folder=build_debug --build=missing -s build_type=Debug
cmake -S . -B build_debug -DCMAKE_TOOLCHAIN_FILE=build_debug/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Debug
cmake --build build_debug --config Debug
pushd build_debug\Debug
.\mad_gui.exe
popd
