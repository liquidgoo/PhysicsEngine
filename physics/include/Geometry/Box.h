#pragma once
#include "SimpleMath.h"
#include "Primitive.h"
#include "Sphere.h"
#include "Plane.h"
#include "..\Collision\Contact.h"

namespace Physics {

    class Box :
        public Primitive
    {
    public:
        DirectX::SimpleMath::Vector3 halfSize;

        void generateVetices(DirectX::SimpleMath::Vector3 vertices[]) const;

        void collideWith(Sphere& other, std::vector<Contact*>& contacts);
        void collideWith(Plane& other, std::vector<Contact*>& contacts);
        void collideWith(Box& other, std::vector<Contact*>& contacts);

        void collideWith(Primitive& other, std::vector<Contact*>& contacts) override;

        Box(DirectX::SimpleMath::Vector3 halfSize);
        Box(DirectX::SimpleMath::Vector3 halfSize, DirectX::SimpleMath::Matrix transform);
    };
}