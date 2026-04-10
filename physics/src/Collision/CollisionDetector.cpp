#include <vector>
#include <limits>
#include "Collision/CollisionDetector.h"
#include "Collision/Contact.h"
#include "Geometry/Primitive.h"
#include "Geometry/Plane.h"
#include "Geometry/Box.h"

    void Physics::CollisionDetector::generateContacts(std::vector<Contact*>& contacts, std::vector<Primitive*> &shapes)
    {
        for (Primitive* shape : shapes) {
            shape->calculateInternals();
        }
        for (int i = 0; i < shapes.size(); i++) {
            for (int j = i + 1; j < shapes.size(); j++) {
                if (shapes[i]->body == shapes[j]->body) continue;
                shapes[i]->collideWith(*shapes[j], contacts);
        }
    }
}