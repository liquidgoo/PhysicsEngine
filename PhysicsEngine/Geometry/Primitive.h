#pragma once
#include "SimpleMath.h"
#include "..\RigidBody.h"
#include "..\Collision\Contact.h"

namespace Physics {

    class Box;
    class Sphere;
    class Plane;

    class Primitive
    {
        
    public:
        RigidBody* body = nullptr;
        DirectX::SimpleMath::Matrix offset;

        DirectX::SimpleMath::Vector3 getAxis(unsigned index) const;
        DirectX::SimpleMath::Matrix getTransform() const;

        void calculateInternals();

        virtual void collideWith(Sphere& other, std::vector<Contact*>& contacts) = 0;
        virtual void collideWith(Plane& other, std::vector<Contact*>& contacts) = 0;
        virtual void collideWith(Box& other, std::vector<Contact*>& contacts) = 0;
        virtual void collideWith(Primitive& other, std::vector<Contact*>& contacts) = 0;

        Primitive(DirectX::SimpleMath::Matrix offset);
        Primitive();

    protected:
        DirectX::SimpleMath::Matrix transform;
    };

}

