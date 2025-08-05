#include "World.h"
#include "RigidBody.h"
#include <iostream>
#include <format>

void Physics::World::step(float length)
{
    for (RigidBody* body : bodies)
    {
        body->integrate(length);
    }

    contacts.clear();

    collisionDetector.generateContacts(contacts, shapes);
    for (Joint* joint : joints)
    {
        joint->addContact(contacts);
    }

    contactResolver.resolveContacts(contacts, length);
    
    for (Contact* contact : contacts)
    {
        delete contact;
    }

}

float Physics::World::getTimeAccumulator()
{
    return timeAccumulator;
}

void Physics::World::update(float delta)
{
    timeAccumulator += delta;
    if (timeAccumulator < stepLength) return;


    while (timeAccumulator >= stepLength)
    {
        step(stepLength);
        timeAccumulator -= stepLength;
    }
}

void Physics::World::registerObject( RigidBody* body)
{
    bodies.push_back(body);
}

void Physics::World::registerObject(RigidBody* body, Primitive* shape)
{
    shape->body = body;
    bodies.push_back(body);
    shapes.push_back(shape);
}

void Physics::World::registerObject(Primitive* shape)
{
    shapes.push_back(shape);
}
    




