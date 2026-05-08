# --- Test Executable ---
# Create a executable for running unit tests
add_executable(unit_tests test_sample.cpp)

# Link the code under test and the GTest runner
target_link_libraries(unit_tests PRIVATE GTest::gtest_main)

# --- Test Discovery ---
# Include the GoogleTest helper module provided by CMake
include(GoogleTest)

if(MINGW)
    gtest_discover_tests(unit_tests DISCOVERY_MODE PRE_TEST)
else()

# Automatically scan 'unit_tests' for TEST() macros and register them with CTest
# PRE_TEST mode configures CTest to run the discovery command right before the actual tests are executed
# instead of immediately after the test executable is built
    gtest_discover_tests(unit_tests)
endif()