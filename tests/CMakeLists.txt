add_executable(mad_unit_tests
  unit/AsciiDataSourceTests.cpp
  unit/EkfUkfTests.cpp
  unit/FilterFactoryTests.cpp
  unit/MadModelTests.cpp
  unit/ParticleFilterTests.cpp
  unit/PipelineTests.cpp
)

target_link_libraries(mad_unit_tests PRIVATE mad_core mad_data mad_pipeline mad_tracking GTest::gtest_main)

target_compile_definitions(mad_unit_tests PRIVATE MAD_REAL_DATA_PATH="${CMAKE_SOURCE_DIR}/data/raw/mad_data.asc")

target_add_warnings(mad_unit_tests)

include(GoogleTest)
gtest_discover_tests(mad_unit_tests)
