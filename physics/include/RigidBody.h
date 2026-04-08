#pragma once
#include "SimpleMath.h"


namespace Physics {

    class RigidBody
    {
    public:
        float inverseMass;
        DirectX::SimpleMath::Matrix inverseInertiaTensor;

        DirectX::SimpleMath::Vector3 position;
        DirectX::SimpleMath::Quaternion orientation;

        DirectX::SimpleMath::Vector3 velocity;
        DirectX::SimpleMath::Vector3 rotation;

        float linearDamping = 0.95;
        float angularDamping = 0.95;

        DirectX::SimpleMath::Vector3 constAcceleration{ 0, -9.8, 0 };

        void setMass(float mass);
        void setInertiaTensor(const DirectX::SimpleMath::Matrix& inertiaTensor);
        
        DirectX::SimpleMath::Matrix getInverseInertiaTensorWorld() const;
        DirectX::SimpleMath::Matrix getTransform() const;

        DirectX::SimpleMath::Vector3 getForceAccum() const;
        DirectX::SimpleMath::Vector3 getTorqueAccum() const;

        void calculateDerivedData();


        void addForce(DirectX::SimpleMath::Vector3&);
        void integrate(float);

        DirectX::SimpleMath::Vector3 lastFrameAcceleration;

        RigidBody();
        RigidBody(float inverseMass, DirectX::SimpleMath::Matrix inverseInertiaTensor);

    protected:
        DirectX::SimpleMath::Matrix inverseInertiaTensorWorld;
        DirectX::SimpleMath::Matrix transformMatrix;

        DirectX::SimpleMath::Vector3 forceAccum;
        DirectX::SimpleMath::Vector3 torqueAccum;

        void clearAccumulators();
    };
}


