#pragma once
#include "MathTypes.h"


namespace Physics {

    class RigidBody
    {
    public:
        float inverseMass;
        Matrix inverseInertiaTensor;

        Vector3 position;
        Quaternion orientation;

        Vector3 velocity;
        Vector3 rotation;

        float linearDamping = 0.95;
        float angularDamping = 0.95;

        Vector3 constAcceleration{ 0, -9.8f, 0 };

        void setMass(float mass);
        void setInertiaTensor(const Matrix& inertiaTensor);
        
        Matrix getInverseInertiaTensorWorld() const;
        Matrix getTransform() const;

        Vector3 getForceAccum() const;
        Vector3 getTorqueAccum() const;

        void calculateDerivedData();


        void addForce(Vector3&);
        void integrate(float);

        Vector3 lastFrameAcceleration;

        RigidBody();
        RigidBody(float inverseMass, Matrix inverseInertiaTensor);

    protected:
        Matrix inverseInertiaTensorWorld;
        Matrix transformMatrix;

        Vector3 forceAccum;
        Vector3 torqueAccum;

        void clearAccumulators();
    };
}


