#include "Collision/Contact.h"
#include <iostream>

Matrix Physics::Contact::getContactToWorld() const
{
    return contactToWorld;
}

Vector3 Physics::Contact::getRelativeContactPosition1() const
{
    return relativeContactPosition1;
}

Vector3 Physics::Contact::getRelativeContactPosition2() const
{
    return relativeContactPosition2;
}

Vector3 Physics::Contact::getContactVelocity() const
{
    return contactVelocity;
}

float Physics::Contact::getDeltaVelocity() const
{
    return deltaVelocity;
}

void Physics::Contact::calculateContactVelocity()
{
    contactVelocity = cross(body1->rotation, relativeContactPosition1);
    contactVelocity += body1->velocity;
    if (body2 != nullptr) {
        contactVelocity -= cross(body2->rotation, relativeContactPosition2); //sign?
        contactVelocity -= body2->velocity;
    }
    contactVelocity = transformVector(contactToWorld.Transpose(), contactVelocity);
}
void Physics::Contact::calculateDeltaVelocity(float duration)
{
    float velocityFromAcceleration = dot(body1->lastFrameAcceleration, contactNormal) * duration;
    if (body2 != nullptr) {
        velocityFromAcceleration -= dot(body2->lastFrameAcceleration, contactNormal) * duration;
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

    contactToWorld = Matrix(
        mathfu::vec4(contactNormal, 0.0f),
        mathfu::vec4(contactTangent[0], 0.0f),
        mathfu::vec4(contactTangent[1], 0.0f),
        mathfu::vec4(0, 0, 0, 1));
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
    matrix = Matrix::Identity();
    matrix(0, 0) = 0.0f;
    matrix(0, 1) = vector.z;
    matrix(0, 2) = -vector.y;
    matrix(1, 0) = -vector.z;
    matrix(1, 1) = 0.0f;
    matrix(1, 2) = vector.x;
    matrix(2, 0) = vector.y;
    matrix(2, 1) = -vector.x;
    matrix(2, 2) = 0.0f;
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
    deltaVelocityByImpulseWorld(3, 3) = 1;

    if (body2 != nullptr)
    {
        inverseMass += body2->inverseMass;

        setSkewSym(impulseToTorque, relativeContactPosition2);
        Matrix deltaVelocityByImpulseWorld2 = impulseToTorque;
        deltaVelocityByImpulseWorld2 *= body2->getInverseInertiaTensorWorld();
        deltaVelocityByImpulseWorld2 *= impulseToTorque;
        deltaVelocityByImpulseWorld2 *= -1;
        deltaVelocityByImpulseWorld2(3, 3) = 1;

        deltaVelocityByImpulseWorld += deltaVelocityByImpulseWorld2;
    }

    Matrix deltaVelocityByImpulse = contactToWorld * deltaVelocityByImpulseWorld * contactToWorld.Transpose();
    deltaVelocityByImpulse(0, 0) += inverseMass;
    deltaVelocityByImpulse(1, 1) += inverseMass;
    deltaVelocityByImpulse(2, 2) += inverseMass;

    Matrix impulseByVelocity = deltaVelocityByImpulse.Inverse();
    //impulseByVelocity._44 = 1;

    Vector3 deltaVelocity{ this->deltaVelocity, -contactVelocity.y, -contactVelocity.z };

    Vector3 impulseContact = transformVector(impulseByVelocity, deltaVelocity);

    float planarImpulse = sqrtf(impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);

    if (planarImpulse > impulseContact.x * friction)
    {
        impulseContact.y /= planarImpulse;
        impulseContact.z /= planarImpulse;

        impulseContact.x = deltaVelocityByImpulse(0, 0) +
            deltaVelocityByImpulse(1, 0) * friction * impulseContact.y +
            deltaVelocityByImpulse(2, 0) * friction * impulseContact.z;
        impulseContact.x = this->deltaVelocity / impulseContact.x;

        impulseContact.y *= friction * impulseContact.x;
        impulseContact.z *= friction * impulseContact.x;
    }
    return impulseContact;
}

Vector3 Physics::Contact::frictionlessImpulse()
{
    Vector3 deltaVelocityByImpulseWorld = cross(relativeContactPosition1, contactNormal);
    deltaVelocityByImpulseWorld = transformVector(body1->getInverseInertiaTensorWorld(), deltaVelocityByImpulseWorld);
    deltaVelocityByImpulseWorld = cross(deltaVelocityByImpulseWorld, relativeContactPosition1);
    
    float deltaVelocityByImpulse = dot(deltaVelocityByImpulseWorld, contactNormal);
    deltaVelocityByImpulse += body1->inverseMass;
    if (body2 != nullptr)
    {
        deltaVelocityByImpulseWorld = cross(relativeContactPosition2, contactNormal);
        deltaVelocityByImpulseWorld = transformVector(body2->getInverseInertiaTensorWorld(), deltaVelocityByImpulseWorld);
        deltaVelocityByImpulseWorld = cross(deltaVelocityByImpulseWorld, relativeContactPosition2);

        deltaVelocityByImpulse += dot(deltaVelocityByImpulseWorld, contactNormal);
        deltaVelocityByImpulse += body2->inverseMass;
    }

    
    return Vector3{ deltaVelocity / deltaVelocityByImpulse, 0,0 };
}


void Physics::Contact::applyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2])
{
    Vector3 impulseContact = frictionImpulse();
    //impulseContact = frictionlessImpulse();
    Vector3 impulseWorld = transformVector(contactToWorld, impulseContact);

    velocityChange[0] = impulseWorld * body1->inverseMass;
    body1->velocity += velocityChange[0];

    Vector3 torque = cross(relativeContactPosition1, impulseWorld);
    rotationChange[0] = transformVector(body1->getInverseInertiaTensorWorld(), torque); //why minus
    body1->rotation += rotationChange[0];

    if (body2 != nullptr) {
        impulseWorld = -impulseWorld;
        velocityChange[1] = impulseWorld * body1->inverseMass;
        body2->velocity += velocityChange[1];

        Vector3 torque = cross(relativeContactPosition2, impulseWorld);
        rotationChange[1] = transformVector(body2->getInverseInertiaTensorWorld(), torque); // why minus
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
                cross(relativeContactPosition[i], contactNormal);
            angularInertiaWorld = transformVector(body->getInverseInertiaTensorWorld(), angularInertiaWorld);
                
                angularInertiaWorld = 
                cross(angularInertiaWorld, relativeContactPosition[i]);
            angularInertia[i] = dot(angularInertiaWorld, contactNormal);


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
                float limit = angularLimit * (relativeContactPosition[i] - contactNormal * dot(relativeContactPosition[i], contactNormal)).Length();
                if (fabsf(angularMove[i]) > limit) {
                    float totalMove = linearMove[i] + angularMove[i];
                    if (angularMove[i] >= 0) angularMove[i] = limit;
                    else angularMove[i] = -limit;
                    linearMove[i] = totalMove - angularMove[i];
                }

                Vector3 torque = cross(relativeContactPosition[i], contactNormal);
                Vector3 impulsePerMove;
                impulsePerMove = transformVector(body->getInverseInertiaTensorWorld(), torque);

                Vector3 rotationPerMove = impulsePerMove / angularInertia[i];
                Vector3 rotation = rotationPerMove * angularMove[i];
                angularChange[i] = rotation;
                body->orientation *= Quaternion::FromEulerAngles(rotation); //order?
            }

            linearChange[i] = linearMove[i] * contactNormal;
            body->position += linearChange[i];
        }

        body = body2;
    }
}

