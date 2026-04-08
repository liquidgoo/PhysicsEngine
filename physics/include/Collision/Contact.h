#pragma once

#include "SimpleMath.h"
#include "..\RigidBody.h"
namespace Physics {

    class Contact
    {
    protected:

        DirectX::SimpleMath::Matrix contactToWorld;

        DirectX::SimpleMath::Vector3 relativeContactPosition1;
        DirectX::SimpleMath::Vector3 relativeContactPosition2;

        DirectX::SimpleMath::Vector3 contactVelocity;
        float deltaVelocity;

    public:

        DirectX::SimpleMath::Vector3 contactPoint;
        DirectX::SimpleMath::Vector3 contactNormal;
        float penetration;
        RigidBody* body1, * body2;


        float restitution =0.4;// const
        float angularLimit = 0.2; //const
        float velocityLimitForRestitution = 0.1; //const
        float friction = 0.2;

        DirectX::SimpleMath::Matrix getContactToWorld() const;
        DirectX::SimpleMath::Vector3 getRelativeContactPosition1() const;
        DirectX::SimpleMath::Vector3 getRelativeContactPosition2() const;
        DirectX::SimpleMath::Vector3 getContactVelocity() const;
        float getDeltaVelocity() const;

        DirectX::SimpleMath::Vector3 frictionImpulse();
        DirectX::SimpleMath::Vector3 frictionlessImpulse();

        void calculateContactVelocity();
        void calculateDeltaVelocity(float duration);
        void calculateContactBasis();
        void calculateInternals(float duration);
        void applyVelocityChange(DirectX::SimpleMath::Vector3[2], DirectX::SimpleMath::Vector3[2]);
        void applyPositionChange(DirectX::SimpleMath::Vector3[2], DirectX::SimpleMath::Vector3[2]);

        friend class ContactResolver;
    };

}