#include "RigidBody.h"
#include <cmath>

namespace Physics {

    void RigidBody::addForce(Vector3& force)
    {
        forceAccum += force;
    }




    static inline void _calculateTransformMatrix(Matrix& transformMatrix,
        const Vector3& position,
        const Quaternion& orientation)
    {
        transformMatrix = Matrix::FromTranslationVector(position) * orientation.ToMatrix4();
    }



    static inline void _transformInertiaTensor(Matrix& iitWorld,//matrix3
        const Quaternion&,
        const Matrix& iitBody, //matrix3
        const Matrix& rotmat) //matrix4
    {
        Matrix rotationOnly = rotmat;
        rotationOnly(0, 3) = 0.0f;
        rotationOnly(1, 3) = 0.0f;
        rotationOnly(2, 3) = 0.0f;
        rotationOnly(3, 0) = 0.0f;
        rotationOnly(3, 1) = 0.0f;
        rotationOnly(3, 2) = 0.0f;
        rotationOnly(3, 3) = 1.0f;
        iitWorld = rotationOnly * iitBody * rotationOnly.Transpose();
    }


    void RigidBody::setInertiaTensor(const Matrix& inertiaTensor)
    {
        inverseInertiaTensor = inertiaTensor.Inverse();
    }

    Matrix RigidBody::getInverseInertiaTensorWorld() const
    {
        return inverseInertiaTensorWorld;
    }

    Matrix RigidBody::getTransform() const
    {
        return transformMatrix;
    }

    Vector3 RigidBody::getForceAccum() const
    {
        return forceAccum;
    }

    Vector3 RigidBody::getTorqueAccum() const
    {
        return torqueAccum;
    }

    void RigidBody::calculateDerivedData()
    {
        _calculateTransformMatrix(transformMatrix, position, orientation);
        _transformInertiaTensor(inverseInertiaTensorWorld, orientation, inverseInertiaTensor, transformMatrix);
    }

    void RigidBody::setMass(float mass)
    {
        this->inverseMass = 1 / mass;//TODO 0 devision
    }

    void RigidBody::clearAccumulators()
    {
        forceAccum = {};
        torqueAccum = {};
    }

    RigidBody::RigidBody()
    {
    }

    RigidBody::RigidBody(float inverseMass, Matrix inerseInertiaTensor)
    {
        //TODO 0 devision
        this->inverseMass = inverseMass;
        this->inverseInertiaTensor = inerseInertiaTensor;
    }



    void RigidBody::integrate(float duration)
    {

        position += velocity * duration;
        orientation = orientation * Quaternion::FromEulerAngles(rotation * duration);
        orientation.Normalize();
        Vector3 acceleration = constAcceleration + forceAccum * inverseMass;
        lastFrameAcceleration = acceleration;
        Vector3 angularAcceleration = transformVector(inverseInertiaTensorWorld, torqueAccum);


        velocity += acceleration * duration;
        rotation += angularAcceleration * duration;


        velocity *= pow(linearDamping, duration);
        rotation *= pow(angularDamping, duration);

        calculateDerivedData();

    }
}