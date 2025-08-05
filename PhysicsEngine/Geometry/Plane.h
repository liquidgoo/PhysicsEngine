#pragma once

#include "SimpleMath.h"
#include "Primitive.h"
#include "..\Collision\Contact.h"

namespace Physics {

    class Sphere;
    class Box;

    class Plane :
        public Primitive
    {
    public:
        DirectX::SimpleMath::Vector3 normal;
        float offset;


        void collideWith(Primitive& other, std::vector<Contact*>& contacts) override;

        void collideWith(Sphere& other, std::vector<Contact*>& contacts);
        void collideWith(Plane& other, std::vector<Contact*>& contacts);
        void collideWith(Box& other, std::vector<Contact*>& contacts);

        Plane(DirectX::SimpleMath::Vector3 normal, float offset);
        Plane(DirectX::SimpleMath::Vector3 normal);

    };
}