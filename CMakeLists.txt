# --- Basic Project Configuration ---
cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable CTest functionality for the project (required for add_test)
enable_testing()

# Automatically name the project based on the directory name
cmake_path(GET CMAKE_SOURCE_DIR FILENAME PROJECT_DIR)
project(${PROJECT_DIR})

# --- Static Analysis: Clang-Tidy ---
# Toggle Clang-Tidy integration (Off by default)
option(ENABLE_CLANG_TIDY "Enable Clang tidy")

if(ENABLE_CLANG_TIDY)
# Search for the clang-tidy executable in the system path
    find_program(CLANG_TIDY_EXE "clang-tidy")
    if(CLANG_TIDY_EXE)
        message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
# Make CMake run clang-tidy during compilation
        set(CMAKE_CXX_CLANG_TIDY ${CLANG_TIDY_EXE})
    else()
        message(WARNING "clang-tidy not found!")
    endif()
endif()

# --- Runtime Analysis: Sanitizers ---
# Toggle Address and Undefined Behavior sanitizers (Off by default)
option(ENABLE_SANITIZERS "Enable Address Sanitizer")

if(ENABLE_SANITIZERS)
# Sanitizers not supported in gcc based mingw
    if(NOT (MINGW AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU"))
# Add instrumentation flags for debugging memory leaks and undefined behavior
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=undefined -fsanitize=address")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined -fsanitize=address")   
    else()
        message("Disabling sanitizers: not supported in gcc based mingw")
    endif()
endif()

# --- Project Structure ---
# Process the source code and testing directories
add_subdirectory(src)
add_subdirectory(tests)