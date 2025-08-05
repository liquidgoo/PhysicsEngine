#pragma once
#include <vector>
#include "Contact.h"
namespace Physics {
    class ContactResolver
    {
    public:
        unsigned velocityIterations = 256;

        unsigned positionIterations = 256;

        float velocityEpsilon = 0.01;

        float positionEpsilon = 0.01;

        unsigned velocityIterationsUsed;

        unsigned positionIterationsUsed;



        void resolveContacts(std::vector<Contact*>& contacts, float duration);
        void prepareContacts(std::vector<Contact*>& contacts, float duration);
        void adjustPositions(std::vector<Contact*>& contacts);
        void adjustVelocities(std::vector<Contact*>& contacts, float duration);
    };

}