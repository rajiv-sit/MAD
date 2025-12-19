@echo off
REM Build MAD in Release configuration.
if not exist build_release (
  mkdir build_release
)
conan install . --output-folder=build_release --build=missing -s build_type=Release
cmake -S . -B build_release -DCMAKE_TOOLCHAIN_FILE=build_release/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build_release --config Release
pushd build_release\Release
.\mad_gui.exe
popd
