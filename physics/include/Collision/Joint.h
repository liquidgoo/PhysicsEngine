#pragma once

#include "RigidBody.h"
#include "Collision/Contact.h"
#include <vector>

namespace Physics
{
    class Joint
    {
    private:
        RigidBody* body[2];
        Vector3 relativeJointPosition[2];
        float maxDisplacement = 0;
        
    public:
        unsigned addContact(std::vector<Contact*> &contacts);

        void set(RigidBody* bodyA, RigidBody* bodyB,
            Vector3 relativeJointPositionA, Vector3 relativeJointPositionB,
            float maxDisplacement);
        void set(RigidBody* bodyA, RigidBody* bodyB,
            Vector3 relativeJointPositionA, Vector3 relativeJointPositionB);
        void set(RigidBody* bodyA,
            Vector3 relativeJointPositionA, Vector3 relativeJointPositionB,
            float maxDisplacement);
        void set(RigidBody* bodyA,
            Vector3 relativeJointPositionA, Vector3 relativeJointPositionB);
    };
}