#include "Geometry/Plane.h"
#include "Geometry/Sphere.h"
#include "Collision/Contact.h"
#include "Collision/IntersectionTests.h"



void Physics::Plane::collideWith(Primitive& other, std::vector<Contact*>& contacts)
{
    other.collideWith(*this, contacts);
}
void Physics::Plane::collideWith(Sphere& other, std::vector<Contact*>& contacts)
{
    IntersectionTests::sphereAndHalfSpace(other, *this, contacts);
}


void Physics::Plane::collideWith(Plane& other, std::vector<Contact*>& contacts)
{
    return;
}
void Physics::Plane::collideWith(Box& other, std::vector<Contact*>& contacts)
{
    IntersectionTests::boxAndHalfSpace(other, *this, contacts);
}

Physics::Plane::Plane(Vector3 normal)
{
    this->normal = normal;
    this->offset = 0;
}
Physics::Plane::Plane(Vector3 normal, float offset ): Plane(normal)
{
    this->offset = offset;
}
