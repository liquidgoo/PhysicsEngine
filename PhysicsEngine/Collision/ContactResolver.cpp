#include "ContactResolver.h"
#include "SimpleMath.h"
#include <iostream>

using namespace DirectX::SimpleMath;


void Physics::ContactResolver::resolveContacts(std::vector<Contact*>& contacts, float duration)
{
    prepareContacts(contacts, duration);
    adjustPositions(contacts);
    adjustVelocities(contacts, duration);
}
void Physics::ContactResolver::prepareContacts(std::vector<Contact*>& contacts, float duration) {
    for (Contact  *contact: contacts)
    {
        contact->calculateInternals(duration);
    }
}
void Physics::ContactResolver::adjustVelocities(std::vector<Contact*>& contacts, float duration)
{
    Vector3 velocityChange[2], rotationChange[2];
    Vector3 deltaVel;


    velocityIterationsUsed = 0;
    while (velocityIterationsUsed < velocityIterations)
    {
        float max = velocityEpsilon;
        unsigned index = -1;
        for (unsigned i = 0; i < contacts.size(); i++)
        {
            if (contacts[i]->deltaVelocity > max)
            {
                max = contacts[i]->deltaVelocity;
                index = i;
            }
        }
        if (index == -1) break;

        contacts[index]->applyVelocityChange(velocityChange, rotationChange);

        for (unsigned i = 0; i < contacts.size(); i++)
        {
            if (contacts[i]->body1 != nullptr)
            {

                if (contacts[i]->body1 == contacts[index]->body1)
                {
                    deltaVel = velocityChange[0] +
                        rotationChange[0].Cross(contacts[i]->relativeContactPosition1);


                    Matrix m = contacts[i]->contactToWorld.Transpose();
                    contacts[i]->contactVelocity += Vector3::Transform(deltaVel, contacts[i]->contactToWorld.Transpose());
                    contacts[i]->calculateDeltaVelocity(duration);
                }
                if (contacts[i]->body1 == contacts[index]->body2)
                {
                    deltaVel = velocityChange[1] +
                        rotationChange[1].Cross(
                            contacts[i]->relativeContactPosition1);

                    Matrix m = contacts[i]->contactToWorld.Transpose();
                    contacts[i]->contactVelocity += Vector3::Transform(deltaVel, contacts[i]->contactToWorld.Transpose());
                    contacts[i]->calculateDeltaVelocity(duration);
                }
            }
            if (contacts[i]->body2 != nullptr)
            {
                if (contacts[i]->body2 == contacts[index]->body1)
                {
                    deltaVel = velocityChange[0] +
                        rotationChange[0].Cross(
                            contacts[i]->relativeContactPosition2);

                    contacts[i]->contactVelocity += -Vector3::Transform(deltaVel, contacts[i]->contactToWorld.Transpose());
                    contacts[i]->calculateDeltaVelocity(duration);
                }
                if (contacts[i]->body2 == contacts[index]->body2)
                {
                    deltaVel = velocityChange[1] +
                        rotationChange[1].Cross(
                            contacts[i]->relativeContactPosition2);

                    contacts[i]->contactVelocity += -Vector3::Transform(deltaVel, contacts[i]->contactToWorld.Transpose());
                    contacts[i]->calculateDeltaVelocity(duration);
                }
            }
        }
        velocityIterationsUsed++;
    }
}
void Physics::ContactResolver::adjustPositions(std::vector<Contact*>& c)
{
    unsigned i, index;
    Vector3 linearChange[2], angularChange[2];
    float max;
    Vector3 cp;

    positionIterationsUsed = 0;
    while (positionIterationsUsed < positionIterations)
    {
        // Find biggest penetration.
        max = positionEpsilon;
        index = -1;
        for (i = 0; i < c.size(); i++) {
            if (c[i]->penetration > max)
            {
                max = c[i]->penetration;
                index = i;
            }
        }
        if (index == -1) break;
        c[index]->applyPositionChange(
            linearChange,
            angularChange);



        for (i = 0; i < c.size(); i++)
        {
            if (c[i]->body1 != nullptr)
            {
                if (c[i]->body1 == c[index]->body1)
                {
                    cp = angularChange[0].Cross(c[i]->relativeContactPosition1);
                    cp += linearChange[0];
                    c[i]->penetration -= cp.Dot(c[i]->contactNormal);
                }
                else if (c[i]->body1 == c[index]->body2)
                {
                    cp = angularChange[1].Cross(c[i]->relativeContactPosition1);
                    cp += linearChange[1];
                    c[i]->penetration -=  cp.Dot(c[i]->contactNormal);
                }
            }
            if (c[i]->body2 != nullptr)
            {
                if (c[i]->body2 == c[index]->body1)
                {
                    cp = angularChange[0].Cross(c[i]->relativeContactPosition2);
                    cp += linearChange[0];
                    c[i]->penetration += cp.Dot(c[i]->contactNormal);
                }
            else if (c[i]->body2 == c[index]->body2)
            {
                cp = angularChange[1].Cross(c[i]->
                    relativeContactPosition2);
                cp += linearChange[1];
                c[i]->penetration += cp.Dot(c[i]-> contactNormal);
            }
            }
        }
        positionIterationsUsed++;
    }
}