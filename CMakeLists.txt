cmake_minimum_required(VERSION 3.20)

project(MAD VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MAD_BUILD_GUI "Build ImGui-based visualizer" ON)
option(MAD_BUILD_TESTS "Build tests" ON)
option(MAD_ENABLE_COVERAGE "Enable coverage target (MSVC OpenCppCoverage)" OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

include(compiler_warnings)

find_package(Eigen3 REQUIRED CONFIG)
find_package(fmt REQUIRED CONFIG)
find_package(spdlog REQUIRED CONFIG)
find_package(nlohmann_json REQUIRED CONFIG)
find_package(glad REQUIRED CONFIG)
find_package(glfw3 REQUIRED CONFIG)
find_package(opengl_system REQUIRED CONFIG)

if(MAD_BUILD_GUI)
  find_package(imgui REQUIRED CONFIG)
  get_target_property(IMGUI_INCLUDE_DIRS imgui::imgui INTERFACE_INCLUDE_DIRECTORIES)
  list(GET IMGUI_INCLUDE_DIRS 0 IMGUI_INCLUDE_DIR)
  set(IMGUI_BACKEND_DIR "${CMAKE_CURRENT_LIST_DIR}/bindings")
endif()

add_library(mad_core
  src/core/Core.cpp
  src/core/EKF.cpp
  src/core/UKF.cpp
  src/core/FilterFactory.cpp
  src/core/ParticleFilter.cpp
  src/core/MadModel.cpp
  src/core/PerformanceMetrics.cpp
)
target_include_directories(mad_core PUBLIC include)
target_link_libraries(mad_core PUBLIC Eigen3::Eigen fmt::fmt spdlog::spdlog nlohmann_json::nlohmann_json)
target_add_warnings(mad_core)

add_library(mad_data
  src/data/Data.cpp
  src/data/AsciiDataSource.cpp
)
target_include_directories(mad_data PUBLIC include)
target_link_libraries(mad_data PUBLIC mad_core)
target_add_warnings(mad_data)

add_library(mad_pipeline
  src/pipeline/Pipeline.cpp
)
target_include_directories(mad_pipeline PUBLIC include)
target_link_libraries(mad_pipeline PUBLIC mad_core mad_data)
target_add_warnings(mad_pipeline)

add_library(mad_tracking
  src/tracking/Tracking.cpp
)
target_include_directories(mad_tracking PUBLIC include)
target_link_libraries(mad_tracking PUBLIC mad_core)
target_add_warnings(mad_tracking)

if(MAD_BUILD_GUI)
  add_library(mad_viz
    src/viz/Visualizer.cpp
  )
  target_include_directories(mad_viz PUBLIC include)
  target_link_libraries(mad_viz PUBLIC mad_core imgui::imgui)
  target_add_warnings(mad_viz)
endif()

add_executable(mad_cli apps/mad_cli/Main.cpp)
target_link_libraries(mad_cli PRIVATE mad_pipeline mad_tracking)

target_add_warnings(mad_cli)

if(MAD_BUILD_GUI)
  add_executable(mad_gui apps/mad_gui/Main.cpp)
  if(EXISTS "${IMGUI_BACKEND_DIR}/imgui_impl_glfw.cpp")
    target_sources(mad_gui PRIVATE
      "${IMGUI_BACKEND_DIR}/imgui_impl_glfw.cpp"
      "${IMGUI_BACKEND_DIR}/imgui_impl_opengl3.cpp"
    )
  endif()
  target_include_directories(mad_gui PRIVATE "${IMGUI_INCLUDE_DIR}" "${IMGUI_BACKEND_DIR}")
  target_link_libraries(mad_gui PRIVATE mad_pipeline mad_tracking mad_viz glfw glad::glad opengl::opengl)
  target_add_warnings(mad_gui)
endif()

if(MAD_BUILD_TESTS)
  find_package(GTest REQUIRED CONFIG)
  enable_testing()
  add_subdirectory(tests)
  if(MAD_ENABLE_COVERAGE)
    find_program(OPEN_CPP_COVERAGE OpenCppCoverage)
    if(OPEN_CPP_COVERAGE)
      add_custom_target(coverage
        COMMAND ${OPEN_CPP_COVERAGE}
          --sources ${CMAKE_SOURCE_DIR}
          --export_type cobertura:coverage.xml
          -- $<TARGET_FILE:mad_unit_tests>
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        DEPENDS mad_unit_tests
      )
    endif()
  endif()
endif()



