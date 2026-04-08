#include "Sphere.h"
#include "..\Collision\Contact.h"
#include "..\Collision\IntersectionTests.h"

void Physics::Sphere::collideWith(Primitive& other, std::vector<Contact*>& contacts)
{
    other.collideWith(*this, contacts);
}
    void Physics::Sphere::collideWith(Sphere& other, std::vector<Contact*>& contacts)
    {
        IntersectionTests::sphereAndSphere(*this, other, contacts);
    }
    void Physics::Sphere::collideWith(Plane& other, std::vector<Contact*>& contacts)
    {
        IntersectionTests::sphereAndHalfSpace(*this, other, contacts);
    }
    void Physics::Sphere::collideWith(Box& other, std::vector<Contact*>& contacts)
    {
        IntersectionTests::boxAndSphere(other, *this, contacts);
    }

    Physics::Sphere::Sphere(float radius)
    {
        this->radius = radius;
    }

    Physics::Sphere::Sphere(float radius, DirectX::SimpleMath::Matrix transform): Primitive(transform)
    {
        this->radius = radius;
    }
