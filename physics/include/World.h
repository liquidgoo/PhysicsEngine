#pragma once
#include <vector>
#include "Geometry/Primitive.h"
#include "RigidBody.h"
#include "Collision/Contact.h"
#include "Collision/ContactResolver.h"
#include "Collision/CollisionDetector.h"
#include "Collision/Joint.h"
namespace Physics {
    class World
    {
    public:
        std::vector<RigidBody*> bodies;
        std::vector<Primitive*> shapes;
        std::vector<Contact*> contacts;
        std::vector<Joint*> joints;

        float stepLength = 1/120.0f;

        float getTimeAccumulator();

        void update(float delta);
        void registerObject(RigidBody* body);
        void registerObject(RigidBody* body, Primitive* shape);
        void registerObject(Primitive* shape);

    protected:
        void step(float length);
        float timeAccumulator = 0.0f;
        CollisionDetector collisionDetector;
        ContactResolver contactResolver;

    };

}