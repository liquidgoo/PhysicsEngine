#pragma once
#include "Geometry/Primitive.h"
#include "Collision/Contact.h"
#include "Geometry/Plane.h"
#include "Geometry/Box.h"
namespace Physics {

    class Sphere :
        public Primitive
    {
    public:

        float radius;

        void collideWith(Primitive& other, std::vector<Contact*>& contacts) override;

        void collideWith(Sphere& other, std::vector<Contact*>& contacts);
        void collideWith(Plane& other, std::vector<Contact*>& contacts);
        void collideWith(Box& other, std::vector<Contact*>& contacts);

       
        Sphere(float radius);
        Sphere(float radius, Matrix transform);

    };

}

