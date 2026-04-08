#include "Joint.h"
#include "SimpleMath.h"

using namespace DirectX::SimpleMath;
unsigned Physics::Joint::addContact(std::vector<Contact*> &contacts)
{
    Vector3 jointPositionA = Vector3::Transform(relativeJointPosition[0], body[0]->getTransform());
    Vector3 jointPositionB = relativeJointPosition[1];
    if (body[1] != nullptr)
    {
        jointPositionB = Vector3::Transform(jointPositionB, body[1]->getTransform());
    }

    Vector3 aToB = jointPositionB - jointPositionA;
    float length = aToB.Length();

    if (length > maxDisplacement)
    {
        Contact* contact = new Contact();
        contact->body1 = body[0];
        contact->body2 = body[1];

        contact->contactNormal = aToB;
        contact->contactNormal.Normalize();

        contact->contactPoint = (jointPositionA + jointPositionB) / 2;
        contact->penetration = length - maxDisplacement;
        
        contact->restitution = 0;
        contact->friction = 1;
        
        contacts.push_back(contact);
        return 1;
    }
    return 0;
}

void Physics::Joint::set(RigidBody* bodyA, RigidBody* bodyB, DirectX::SimpleMath::Vector3 relativeJointPositionA, DirectX::SimpleMath::Vector3 relativeJointPositionB, float maxDisplacement)
{
    body[0] = bodyA;
    body[1] = bodyB;
    relativeJointPosition[0] = relativeJointPositionA;
    relativeJointPosition[1] = relativeJointPositionB;
    this->maxDisplacement = maxDisplacement;
}

void Physics::Joint::set(RigidBody* bodyA, RigidBody* bodyB, DirectX::SimpleMath::Vector3 relativeJointPositionA, DirectX::SimpleMath::Vector3 relativeJointPositionB)
{
    set(bodyA, bodyB, relativeJointPositionA, relativeJointPositionB, 0);
}

void Physics::Joint::set(RigidBody* bodyA, DirectX::SimpleMath::Vector3 relativeJointPositionA, DirectX::SimpleMath::Vector3 relativeJointPositionB, float maxDisplacement)
{
    set(bodyA, nullptr, relativeJointPositionA, relativeJointPositionB, maxDisplacement);
}


void Physics::Joint::set(RigidBody* bodyA, DirectX::SimpleMath::Vector3 relativeJointPositionA, DirectX::SimpleMath::Vector3 relativeJointPositionB)
{
    set(bodyA, nullptr, relativeJointPositionA, relativeJointPositionB, 0);
}
