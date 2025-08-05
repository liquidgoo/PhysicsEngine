#include "RigidBody.h"
#include <cmath>

using namespace DirectX::SimpleMath;
namespace Physics {

    void RigidBody::addForce(Vector3& force)
    {
        forceAccum += force;
    }




    static inline void _calculateTransformMatrix(Matrix& transformMatrix,
        const Vector3& position,
        const Quaternion& orientation)
    {
        transformMatrix = Matrix::CreateFromQuaternion(orientation);
        transformMatrix._41 = position.x;
        transformMatrix._42 = position.y;
        transformMatrix._43 = position.z;
    }



    static inline void _transformInertiaTensor(Matrix& iitWorld,//matrix3
        const Quaternion& q,
        const Matrix& iitBody, //matrix3
        const Matrix& rotmat) //matrix4
    {
        float t4 = rotmat._11 * iitBody._11 +
            rotmat._21 * iitBody._12 +
            rotmat._31 * iitBody._13;
        float t9 = rotmat._11 * iitBody._21 +
            rotmat._21 * iitBody._22 +
            rotmat._31 * iitBody._23;
        float t14 = rotmat._11 * iitBody._31 +
            rotmat._21 * iitBody._32 +
            rotmat._31 * iitBody._33;
        float t28 = rotmat._12 * iitBody._11 +
            rotmat._22 * iitBody._12 +
            rotmat._32 * iitBody._13;
        float t33 = rotmat._12 * iitBody._21 +
            rotmat._22 * iitBody._22 +
            rotmat._32 * iitBody._23;
        float t38 = rotmat._12 * iitBody._31 +
            rotmat._22 * iitBody._32 +
            rotmat._32 * iitBody._33;
        float t52 = rotmat._13 * iitBody._11 +
            rotmat._23 * iitBody._12 +
            rotmat._33 * iitBody._13;
        float t57 = rotmat._13 * iitBody._21 +
            rotmat._23 * iitBody._22 +
            rotmat._33 * iitBody._23;
        float t62 = rotmat._13 * iitBody._31 +
            rotmat._23 * iitBody._32 +
            rotmat._33 * iitBody._33;
        iitWorld._11 = t4 * rotmat._11 +
            t9 * rotmat._21 +
            t14 * rotmat._31;
        iitWorld._21 = t4 * rotmat._12 +
            t9 * rotmat._22 +
            t14 * rotmat._32;
        iitWorld._31 = t4 * rotmat._13 +
            t9 * rotmat._23 +
            t14 * rotmat._33;
        iitWorld._12 = t28 * rotmat._11 +
            t33 * rotmat._21 +
            t38 * rotmat._31;
        iitWorld._22 = t28 * rotmat._12 +
            t33 * rotmat._22 +
            t38 * rotmat._32;
        iitWorld._32 = t28 * rotmat._13 +
            t33 * rotmat._23 +
            t38 * rotmat._33;
        iitWorld._13 = t52 * rotmat._11 +
            t57 * rotmat._21 +
            t62 * rotmat._31;
        iitWorld._23 = t52 * rotmat._12 +
            t57 * rotmat._22 +
            t62 * rotmat._32;
        iitWorld._33 = t52 * rotmat._13 +
            t57 * rotmat._23 +
            t62 * rotmat._33;
    }


    void RigidBody::setInertiaTensor(const Matrix& inertiaTensor)
    {
        inertiaTensor.Invert(inverseInertiaTensor);
    }

    DirectX::SimpleMath::Matrix RigidBody::getInverseInertiaTensorWorld() const
    {
        return inverseInertiaTensorWorld;
    }

    DirectX::SimpleMath::Matrix RigidBody::getTransform() const
    {
        return transformMatrix;
    }

    DirectX::SimpleMath::Vector3 RigidBody::getForceAccum() const
    {
        return forceAccum;
    }

    DirectX::SimpleMath::Vector3 RigidBody::getTorqueAccum() const
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

    RigidBody::RigidBody(float inverseMass, DirectX::SimpleMath::Matrix inerseInertiaTensor)
    {
        //TODO 0 devision
        this->inverseMass = inverseMass;
        this->inverseInertiaTensor = inerseInertiaTensor;
    }



    void RigidBody::integrate(float duration)
    {

        position += velocity * duration;
        orientation = orientation * Quaternion::CreateFromYawPitchRoll(rotation * duration);
        Vector3 acceleration = constAcceleration + forceAccum * inverseMass;
        lastFrameAcceleration = acceleration;
        Vector3 angularAcceleration; 
        torqueAccum.Transform(angularAcceleration, inverseInertiaTensorWorld, angularAcceleration);


        velocity += acceleration * duration;
        rotation += angularAcceleration * duration;


        velocity *= pow(linearDamping, duration);
        rotation *= pow(angularDamping, duration);

        calculateDerivedData();

    }
}