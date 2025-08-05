#pragma once

#include "SimpleMath.h"
#include "..\RigidBody.h"
#include "Contact.h"
#include <vector>

namespace Physics
{
    class Joint
    {
    private:
        RigidBody* body[2];
        DirectX::SimpleMath::Vector3 relativeJointPosition[2];
        float maxDisplacement = 0;
        
    public:
        unsigned addContact(std::vector<Contact*> &contacts);

        void set(RigidBody* bodyA, RigidBody* bodyB,
            DirectX::SimpleMath::Vector3 relativeJointPositionA, DirectX::SimpleMath::Vector3 relativeJointPositionB,
            float maxDisplacement);
        void set(RigidBody* bodyA, RigidBody* bodyB,
            DirectX::SimpleMath::Vector3 relativeJointPositionA, DirectX::SimpleMath::Vector3 relativeJointPositionB);
        void set(RigidBody* bodyA,
            DirectX::SimpleMath::Vector3 relativeJointPositionA, DirectX::SimpleMath::Vector3 relativeJointPositionB,
            float maxDisplacement);
        void set(RigidBody* bodyA,
            DirectX::SimpleMath::Vector3 relativeJointPositionA, DirectX::SimpleMath::Vector3 relativeJointPositionB);
    };
}