#pragma once

#include "RigidBody.h"

namespace Physics {

    class Contact
    {
    protected:

        Matrix contactToWorld;

        Vector3 relativeContactPosition1;
        Vector3 relativeContactPosition2;

        Vector3 contactVelocity;
        float deltaVelocity;

    public:

        Vector3 contactPoint;
        Vector3 contactNormal;
        float penetration;
        RigidBody* body1, * body2;


        float restitution =0.4;// const
        float angularLimit = 0.2; //const
        float velocityLimitForRestitution = 0.1; //const
        float friction = 0.2;

        Matrix getContactToWorld() const;
        Vector3 getRelativeContactPosition1() const;
        Vector3 getRelativeContactPosition2() const;
        Vector3 getContactVelocity() const;
        float getDeltaVelocity() const;

        Vector3 frictionImpulse();
        Vector3 frictionlessImpulse();

        void calculateContactVelocity();
        void calculateDeltaVelocity(float duration);
        void calculateContactBasis();
        void calculateInternals(float duration);
        void applyVelocityChange(Vector3[2], Vector3[2]);
        void applyPositionChange(Vector3[2], Vector3[2]);

        friend class ContactResolver;
    };

}