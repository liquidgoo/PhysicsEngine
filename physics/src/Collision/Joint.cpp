#include "Collision/Joint.h"
unsigned Physics::Joint::addContact(std::vector<Contact*> &contacts)
{
    Vector3 jointPositionA = transformPoint(body[0]->getTransform(), relativeJointPosition[0]);
    Vector3 jointPositionB = relativeJointPosition[1];
    if (body[1] != nullptr)
    {
        jointPositionB = transformPoint(body[1]->getTransform(), jointPositionB);
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

void Physics::Joint::set(RigidBody* bodyA, RigidBody* bodyB, Vector3 relativeJointPositionA, Vector3 relativeJointPositionB, float maxDisplacement)
{
    body[0] = bodyA;
    body[1] = bodyB;
    relativeJointPosition[0] = relativeJointPositionA;
    relativeJointPosition[1] = relativeJointPositionB;
    this->maxDisplacement = maxDisplacement;
}

void Physics::Joint::set(RigidBody* bodyA, RigidBody* bodyB, Vector3 relativeJointPositionA, Vector3 relativeJointPositionB)
{
    set(bodyA, bodyB, relativeJointPositionA, relativeJointPositionB, 0);
}

void Physics::Joint::set(RigidBody* bodyA, Vector3 relativeJointPositionA, Vector3 relativeJointPositionB, float maxDisplacement)
{
    set(bodyA, nullptr, relativeJointPositionA, relativeJointPositionB, maxDisplacement);
}


void Physics::Joint::set(RigidBody* bodyA, Vector3 relativeJointPositionA, Vector3 relativeJointPositionB)
{
    set(bodyA, nullptr, relativeJointPositionA, relativeJointPositionB, 0);
}
