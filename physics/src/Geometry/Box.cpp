#include "Box.h"
#include "SimpleMath.h"
#include "Primitive.h"
#include "..\Collision\IntersectionTests.h"


namespace Physics {

    void Box::generateVetices(DirectX::SimpleMath::Vector3 vertices[]) const
    {

        vertices[0] = DirectX::SimpleMath::Vector3(-halfSize.x, -halfSize.y, -halfSize.z);
        vertices[1] = DirectX::SimpleMath::Vector3(-halfSize.x, -halfSize.y, +halfSize.z);
        vertices[2] = DirectX::SimpleMath::Vector3(-halfSize.x, +halfSize.y, -halfSize.z);
        vertices[3] = DirectX::SimpleMath::Vector3(-halfSize.x, +halfSize.y, +halfSize.z);
        vertices[4] = DirectX::SimpleMath::Vector3(+halfSize.x, -halfSize.y, -halfSize.z);
        vertices[5] = DirectX::SimpleMath::Vector3(+halfSize.x, -halfSize.y, +halfSize.z);
        vertices[6] = DirectX::SimpleMath::Vector3(+halfSize.x, +halfSize.y, -halfSize.z);
        vertices[7] = DirectX::SimpleMath::Vector3(+halfSize.x, +halfSize.y, +halfSize.z);

        const DirectX::SimpleMath::Matrix& offset = this->transform;

        for (int i = 0; i < 8; i++) {
            DirectX::SimpleMath::Vector3::Transform(vertices[i], offset, vertices[i]);
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

    Physics::Box::Box(DirectX::SimpleMath::Vector3 halfSize)
    {
        this->halfSize = halfSize;
    }

    Physics::Box::Box(DirectX::SimpleMath::Vector3 halfSize, DirectX::SimpleMath::Matrix transform) : Primitive(transform)
    {
        this->halfSize = halfSize;
    }

}
