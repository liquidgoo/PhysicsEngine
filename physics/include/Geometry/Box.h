#pragma once
#include "Geometry/Primitive.h"
#include "Geometry/Sphere.h"
#include "Geometry/Plane.h"
#include "Collision/Contact.h"

namespace Physics {

    class Box :
        public Primitive
    {
    public:
        Vector3 halfSize;

        void generateVetices(Vector3 vertices[]) const;

        void collideWith(Sphere& other, std::vector<Contact*>& contacts);
        void collideWith(Plane& other, std::vector<Contact*>& contacts);
        void collideWith(Box& other, std::vector<Contact*>& contacts);

        void collideWith(Primitive& other, std::vector<Contact*>& contacts) override;

        Box(Vector3 halfSize);
        Box(Vector3 halfSize, Matrix transform);
    };
}