#pragma once

#include <vector>
#include "Collision/Contact.h"
#include "Geometry/Primitive.h"

namespace Physics {

    class CollisionDetector
    {
    public:

        void generateContacts(std::vector<Contact*>& contacts, std::vector<Primitive*> &shapes);
        
    };

}