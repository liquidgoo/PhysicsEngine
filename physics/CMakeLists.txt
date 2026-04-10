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

find_package(mathfu REQUIRED)

add_library(physics_lib STATIC ${ENGINE_SOURCES} ${ENGINE_HEADERS})
target_include_directories(physics_lib PUBLIC include)

target_link_libraries(physics_lib PRIVATE mathfu::mathfu)