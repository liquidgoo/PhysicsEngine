#pragma once
#include <vector>
#include "MathTypes.h"
#include "RigidBody.h"
#include "Collision/Contact.h"

namespace Physics {

    class Box;
    class Sphere;
    class Plane;

    class Primitive
    {
        
    public:
        RigidBody* body = nullptr;
        Matrix offset;

        Vector3 getAxis(unsigned index) const;
        Matrix getTransform() const;

        void calculateInternals();

        virtual void collideWith(Sphere& other, std::vector<Contact*>& contacts) = 0;
        virtual void collideWith(Plane& other, std::vector<Contact*>& contacts) = 0;
        virtual void collideWith(Box& other, std::vector<Contact*>& contacts) = 0;
        virtual void collideWith(Primitive& other, std::vector<Contact*>& contacts) = 0;

        Primitive(Matrix offset);
        Primitive();

    protected:
        Matrix transform;
    };

}

