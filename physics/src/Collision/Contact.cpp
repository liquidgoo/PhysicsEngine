#include "Contact.h"
#include "SimpleMath.h"

using namespace DirectX::SimpleMath;
#include <iostream>

DirectX::SimpleMath::Matrix Physics::Contact::getContactToWorld() const
{
    return contactToWorld;
}

DirectX::SimpleMath::Vector3 Physics::Contact::getRelativeContactPosition1() const
{
    return relativeContactPosition1;
}

DirectX::SimpleMath::Vector3 Physics::Contact::getRelativeContactPosition2() const
{
    return relativeContactPosition2;
}

DirectX::SimpleMath::Vector3 Physics::Contact::getContactVelocity() const
{
    return contactVelocity;
}

float Physics::Contact::getDeltaVelocity() const
{
    return deltaVelocity;
}

void Physics::Contact::calculateContactVelocity()
{
    contactVelocity = body1->rotation.Cross(relativeContactPosition1);
    contactVelocity += body1->velocity;
    if (body2 != nullptr) {
        contactVelocity -= body2->rotation.Cross(relativeContactPosition2); //sign?
        contactVelocity -= body2->velocity;
    }
    Matrix matrix = contactToWorld.Transpose();
    Vector3::Transform(contactVelocity, matrix, contactVelocity);
}
void Physics::Contact::calculateDeltaVelocity(float duration)
{
    float velocityFromAcceleration = body1->lastFrameAcceleration.Dot(contactNormal) * duration;
    if (body2 != nullptr) {
        velocityFromAcceleration -= body2->lastFrameAcceleration.Dot(contactNormal) * duration;
    }
    float restitution = (contactVelocity.Length() < velocityLimitForRestitution) ? 0 : this->restitution;
    
    deltaVelocity = -contactVelocity.x - restitution * (contactVelocity.x - velocityFromAcceleration);
}
void Physics::Contact::calculateContactBasis()
{
    Vector3 contactTangent[2];

    
    if (fabsf(contactNormal.x) > fabsf(contactNormal.y))
    {

        const float s = (float)1.0f / fabsf(contactNormal.z * contactNormal.z +
            contactNormal.x * contactNormal.x);

        contactTangent[0].x = contactNormal.z * s;
        contactTangent[0].y = 0;
        contactTangent[0].z = -contactNormal.x * s;

        contactTangent[1].x = contactNormal.y * contactTangent[0].x;
        contactTangent[1].y = contactNormal.z * contactTangent[0].x -
            contactNormal.x * contactTangent[0].z;
        contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
    }
    else
    {
        const float s = (float)1.0 / fabsf(contactNormal.z * contactNormal.z +
            contactNormal.y * contactNormal.y);

        contactTangent[0].x = 0;
        contactTangent[0].y = -contactNormal.z * s;
        contactTangent[0].z = contactNormal.y * s;

        contactTangent[1].x = contactNormal.y * contactTangent[0].z -
            contactNormal.z * contactTangent[0].y;
        contactTangent[1].y = -contactNormal.x * contactTangent[0].z;
        contactTangent[1].z = contactNormal.x * contactTangent[0].y;
    }

    contactToWorld = Matrix{ contactNormal, contactTangent[0], contactTangent[1] };
}

void Physics::Contact::calculateInternals(float duration)
{
    if (body1 == nullptr) {
        RigidBody* temp = body2;
        body2 = body1;
        body1 = temp;
        contactNormal = -contactNormal;
    }

    calculateContactBasis();

    relativeContactPosition1 = contactPoint - body1->position;
    if (body2 != nullptr) relativeContactPosition2 = contactPoint - body2->position;

    calculateContactVelocity();
    calculateDeltaVelocity(duration);
}

void setSkewSym(Matrix& matrix, Vector3& vector)
{
    matrix._11 = 0;
    matrix._12 = vector.z;
    matrix._13 = -vector.y;
    matrix._14 = 0;
    matrix._21 = -vector.z;
    matrix._22 = 0;
    matrix._23 = vector.x;
    matrix._31 = vector.y;
    matrix._32 = -vector.x;
    matrix._33 = 0;
    matrix._34 = 0;
}
Vector3 Physics::Contact::frictionImpulse()
{
    float inverseMass = body1->inverseMass;

    Matrix impulseToTorque;
    setSkewSym(impulseToTorque, relativeContactPosition1);
    Matrix deltaVelocityByImpulseWorld = impulseToTorque;
    deltaVelocityByImpulseWorld *= body1->getInverseInertiaTensorWorld();
    deltaVelocityByImpulseWorld *= impulseToTorque;
    deltaVelocityByImpulseWorld *= -1;
    deltaVelocityByImpulseWorld._44 = 1;

    if (body2 != nullptr)
    {
        inverseMass += body2->inverseMass;

        setSkewSym(impulseToTorque, relativeContactPosition2);
        Matrix deltaVelocityByImpulseWorld2 = impulseToTorque;
        deltaVelocityByImpulseWorld2 *= body2->getInverseInertiaTensorWorld();
        deltaVelocityByImpulseWorld2 *= impulseToTorque;
        deltaVelocityByImpulseWorld2 *= -1;
        deltaVelocityByImpulseWorld2._44 = 1;

        deltaVelocityByImpulseWorld += deltaVelocityByImpulseWorld2;
    }

    Matrix deltaVelocityByImpulse = contactToWorld * deltaVelocityByImpulseWorld * contactToWorld.Transpose();
    deltaVelocityByImpulse._11 += inverseMass;
    deltaVelocityByImpulse._22 += inverseMass;
    deltaVelocityByImpulse._33 += inverseMass;

    Matrix impulseByVelocity = deltaVelocityByImpulse.Invert();
    //impulseByVelocity._44 = 1;

    Vector3 deltaVelocity{ this->deltaVelocity, -contactVelocity.y, -contactVelocity.z };

    Vector3 impulseContact = Vector3::Transform(deltaVelocity, impulseByVelocity);

    float planarImpulse = sqrtf(impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);

    if (planarImpulse > impulseContact.x * friction)
    {
        impulseContact.y /= planarImpulse;
        impulseContact.z /= planarImpulse;

        impulseContact.x = deltaVelocityByImpulse._11 +
            deltaVelocityByImpulse._21 * friction * impulseContact.y +
            deltaVelocityByImpulse._31 * friction * impulseContact.z;
        impulseContact.x = this->deltaVelocity / impulseContact.x;

        impulseContact.y *= friction * impulseContact.x;
        impulseContact.z *= friction * impulseContact.x;
    }
    return impulseContact;
}

Vector3 Physics::Contact::frictionlessImpulse()
{
    Vector3 deltaVelocityByImpulseWorld = relativeContactPosition1.Cross(contactNormal);
    deltaVelocityByImpulseWorld = Vector3::Transform(deltaVelocityByImpulseWorld, body1->getInverseInertiaTensorWorld());
    deltaVelocityByImpulseWorld = deltaVelocityByImpulseWorld.Cross(relativeContactPosition1);
    
    float deltaVelocityByImpulse = deltaVelocityByImpulseWorld.Dot(contactNormal);
    deltaVelocityByImpulse += body1->inverseMass;
    if (body2 != nullptr)
    {
        deltaVelocityByImpulseWorld = relativeContactPosition2.Cross(contactNormal);
        deltaVelocityByImpulseWorld = Vector3::Transform(deltaVelocityByImpulseWorld, body2->getInverseInertiaTensorWorld());
        deltaVelocityByImpulseWorld = deltaVelocityByImpulseWorld.Cross(relativeContactPosition2);

        deltaVelocityByImpulse += deltaVelocityByImpulseWorld.Dot(contactNormal);
        deltaVelocityByImpulse += body2->inverseMass;
    }

    
    return Vector3{ deltaVelocity / deltaVelocityByImpulse, 0,0 };
}


void Physics::Contact::applyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2])
{
    Vector3 impulseContact = frictionImpulse();
    //impulseContact = frictionlessImpulse();
    Vector3 impulseWorld = Vector3::Transform(impulseContact, contactToWorld);

    velocityChange[0] = impulseWorld * body1->inverseMass;
    body1->velocity += velocityChange[0];

    Vector3 torque = relativeContactPosition1.Cross(impulseWorld);
    rotationChange[0] = Vector3::Transform(torque, body1->getInverseInertiaTensorWorld()); //why minus
    body1->rotation += rotationChange[0];

    if (body2 != nullptr) {
        impulseWorld = -impulseWorld;
        velocityChange[1] = impulseWorld * body1->inverseMass;
        body2->velocity += velocityChange[1];

        Vector3 torque = relativeContactPosition2.Cross(impulseWorld);
        rotationChange[1] = Vector3::Transform(torque, body2->getInverseInertiaTensorWorld()); // why minus
        body2->rotation += rotationChange[1];
    }
}

void Physics::Contact::applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2])
{
    float linearInertia[2];
    float angularInertia[2];
    float totalInertia = 0;
    Vector3 relativeContactPosition[2] = { relativeContactPosition1, relativeContactPosition2 };

    RigidBody* body = body1;
    for (int i = 0; i < 2; i++) {
        if (body != nullptr)
        {
            Vector3 angularInertiaWorld =
                relativeContactPosition[i].Cross(contactNormal);
            angularInertiaWorld = Vector3::Transform(angularInertiaWorld, body->getInverseInertiaTensorWorld());
                
            angularInertiaWorld = 
                angularInertiaWorld.Cross(relativeContactPosition[i]);
            angularInertia[i] = angularInertiaWorld.Dot(contactNormal);


            //angularInertia[i] = 0;

            linearInertia[i] = body->inverseMass;

            totalInertia += linearInertia[i] + angularInertia[i];
        }
        body = body2;
    }

    float inverseInertia = 1 / totalInertia;
    float linearMove[2];
    float angularMove[2];
    linearMove[0] = penetration * linearInertia[0] * inverseInertia;
    angularMove[0] = penetration * angularInertia[0] * inverseInertia;

    if (body2 != nullptr) {
        linearMove[1] = -penetration * linearInertia[1] * inverseInertia;
        angularMove[1] = -penetration * angularInertia[1] * inverseInertia;
    }


    body = body1;
    for (int i = 0; i < 2; i++) {

        if (body != nullptr) {

            if (angularMove[i] == 0) { angularChange[i] = {}; }
            else {
                float limit = angularLimit * (relativeContactPosition[i] - contactNormal * relativeContactPosition[i].Dot(contactNormal)).Length();
                if (fabsf(angularMove[i]) > limit) {
                    float totalMove = linearMove[i] + angularMove[i];
                    if (angularMove[i] >= 0) angularMove[i] = limit;
                    else angularMove[i] = -limit;
                    linearMove[i] = totalMove - angularMove[i];
                }

                Vector3 torque = relativeContactPosition[i].Cross(contactNormal);
                Vector3 impulsePerMove;
                Vector3::Transform(torque, body->getInverseInertiaTensorWorld(), impulsePerMove);

                Vector3 rotationPerMove = impulsePerMove / angularInertia[i];
                Vector3 rotation = rotationPerMove * angularMove[i];
                angularChange[i] = rotation;
                body->orientation *= Quaternion::CreateFromYawPitchRoll(rotation); //order?
            }

            linearChange[i] = linearMove[i] * contactNormal;
            body->position += linearChange[i];
        }

        body = body2;
    }
}

