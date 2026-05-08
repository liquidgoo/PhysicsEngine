# --- Target: Sample Library ---
# Create a static library from the source files
set(ENGINE_SOURCES
    src/RigidBody.cpp
    src/World.cpp
    src/Collision/CollisionDetector.cpp
    src/Collision/ContactResolver.cpp
    src/Collision/IntersectionTests.cpp
    src/Collision/Joint.cpp
    src/Geometry/Box.cpp
    src/Geometry/Plane.cpp
    src/Geometry/Primitive.cpp
    src/Geometry/Sphere.cpp
)

set(ENGINE_HEADERS
    include/Physics.h
    include/RigidBody.h
    include/World.h
    include/Collision/CollisionDetector.h
    include/Collision/ContactResolver.h
    include/Collision/IntersectionTests.h
    include/Collision/Joint.h
    include/Geometry/Box.h
    include/Geometry/Plane.h
    include/Geometry/Primitive.h
    include/Geometry/Sphere.h
)

add_library(physics_lib STATIC ${ENGINE_SOURCES} ${ENGINE_HEADERS})
target_include_directories(physics_lib PUBLIC include)

add_executable(physics_app main.cpp)

target_link_libraries(physics_lib PRIVATE mathfu::mathfu)
target_link_libraries(physics_app PRIVATE 
    physics_lib 
    glad::glad
    glfw
    opengl::opengl
)

# Copy MinGW runtime DLLs to output directory after build
if(MINGW)
    # Get MinGW bin directory from compiler path
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libstdc++-6.dll
        OUTPUT_VARIABLE LIBSTDCPP_DLL
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libgcc_s_seh-1.dll
        OUTPUT_VARIABLE LIBGCC_DLL
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    set(MINGW_DLLS
        "${LIBSTDCPP_DLL}"
        "${LIBGCC_DLL}"
        "/usr/x86_64-w64-mingw32/lib/libwinpthread-1.dll"
    )

    foreach(DLL ${MINGW_DLLS})
        if(EXISTS ${DLL})
            add_custom_command(TARGET physics_app POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    ${DLL}
                    $<TARGET_FILE_DIR:physics_app>
                COMMENT "Copying ${DLL}"
            )
        endif()
    endforeach()
endif()