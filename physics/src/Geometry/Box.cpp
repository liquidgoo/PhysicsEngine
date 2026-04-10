#include "Geometry/Box.h"
#include "Geometry/Primitive.h"
#include "Collision/IntersectionTests.h"


namespace Physics {

    void Box::generateVetices(Vector3 vertices[]) const
    {

        vertices[0] = Vector3(-halfSize.x, -halfSize.y, -halfSize.z);
        vertices[1] = Vector3(-halfSize.x, -halfSize.y, +halfSize.z);
        vertices[2] = Vector3(-halfSize.x, +halfSize.y, -halfSize.z);
        vertices[3] = Vector3(-halfSize.x, +halfSize.y, +halfSize.z);
        vertices[4] = Vector3(+halfSize.x, -halfSize.y, -halfSize.z);
        vertices[5] = Vector3(+halfSize.x, -halfSize.y, +halfSize.z);
        vertices[6] = Vector3(+halfSize.x, +halfSize.y, -halfSize.z);
        vertices[7] = Vector3(+halfSize.x, +halfSize.y, +halfSize.z);

        const Matrix& offset = this->transform;

        for (int i = 0; i < 8; i++) {
            vertices[i] = transformPoint(offset, vertices[i]);
        }
    }



    void Box::collideWith(Sphere& other, std::vector<Contact*>& contacts)
    {
        IntersectionTests::boxAndSphere(*this, other, contacts);
    }
    void Box::collideWith(Plane& other, std::vector<Contact*>& contacts)
    {
        IntersectionTests::boxAndHalfSpace(*this, other, contacts);
    }
    void Box::collideWith(Box& other, std::vector<Contact*>& contacts)
    {
        IntersectionTests::boxAndBox(*this, other, contacts);
    }

    void Box::collideWith(Primitive& other, std::vector<Contact*>& contacts)
    {
        other.collideWith(*this, contacts);
    }

    Physics::Box::Box(Vector3 halfSize)
    {
        this->halfSize = halfSize;
    }

    Physics::Box::Box(Vector3 halfSize, Matrix transform) : Primitive(transform)
    {
        this->halfSize = halfSize;
    }

}
