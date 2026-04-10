#include "Geometry/Sphere.h"
#include "Collision/Contact.h"
#include "Geometry/Plane.h"
#include "Geometry/Box.h"

namespace Physics {
    namespace IntersectionTests
    {
         extern float tolerance;
         unsigned sphereAndSphere(const Sphere& one, const Sphere& two, std::vector<Contact*>& contacts);
         unsigned sphereAndHalfSpace(const Sphere& sphere, const Plane& plane, std::vector<Contact*>& contacts);
         unsigned boxAndHalfSpace(const Box& box, const Plane& plane, std::vector<Contact*>& contacts);
         unsigned boxAndSphere(const Box& box, const Sphere& sphere, std::vector<Contact*>& contacts);
         unsigned boxAndBox(const Box& one, const Box& two, std::vector<Contact*>& contacts);
    };
}