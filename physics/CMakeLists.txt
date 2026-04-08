# --- Target: Sample Library ---
# Create a static library from the source files
set(ENGINE_SOURCES
    src/RigidBody.cpp
    src/World.cpp
    src/Collision/CollisionDetector.cpp
    src/Collision/ContactResolver.cpp
    src/Collision/IntersectionTests.cpp
    src/Collision/Joint.cpp
    src/Box.cpp
    src/Plane.cpp
    src/Primitive.cpp
    src/Sphere.cpp
)

set(ENGINE_HEADERS
    src/Physics.h
    src/RigidBody.h
    src/World.h
    src/Collision/CollisionDetector.h
    src/Collision/ContactResolver.h
    src/Collision/IntersectionTests.h
    src/Collision/Joint.v
    src/Box.h
    src/Plane.h
    src/Primitive.h
    src/Sphere.h
)

find_package(mathfu REQUIRED)

add_library(physics_lib STATIC ${ENGINE_SOURCES} ${ENGINE_HEADERS})
target_include_directories(physics_lib PUBLIC ${ENGINE_HEADERS})

target_link_libraries(physics_lib PRIVATE mathfu::mathfu)