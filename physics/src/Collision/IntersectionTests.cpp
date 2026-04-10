#include "Geometry/Sphere.h"
#include "Geometry/Box.h"
#include "Geometry/Plane.h"
#include "Collision/IntersectionTests.h"

namespace Physics {

    float IntersectionTests::tolerance = 0.0f;
    unsigned IntersectionTests::sphereAndSphere(
        const Sphere& one,
        const Sphere& two,
        std::vector<Contact*>& contacts)
    {
        Vector3 positionOne = one.getAxis(3);
        Vector3 positionTwo = two.getAxis(3);
        Vector3 midline = positionOne - positionTwo;
        float size = midline.Length();
        if (size <= 0.0f || size >= one.radius + two.radius)
        {
            return 0;
        }
        Vector3 normal = midline / size;
        Contact* contact = new Contact;
        contact->contactNormal = normal;
        contact->contactPoint = positionOne + midline * 0.5f;
        contact->penetration = (one.radius + two.radius - size);
        contact->body1 = one.body;
        contact->body2 = two.body;
        contacts.push_back(contact);
        return 1;
    }


    unsigned IntersectionTests::sphereAndHalfSpace(
        const Sphere& sphere,
        const Physics::Plane& plane,
        std::vector<Contact*>& contacts
    )
    {
        Vector3 position = sphere.getAxis(3);
        float ballDistance =
            dot(plane.normal, position) -
            sphere.radius - plane.offset;
        if (ballDistance >= 0) return 0;
        Contact* contact = new Contact;
        contact->contactNormal = plane.normal;
        contact->penetration = -ballDistance;
        contact->contactPoint =
            position - plane.normal * (ballDistance + sphere.radius);
        contact->body1 = sphere.body;
        contact->body2 = plane.body;
        contacts.push_back(contact);
        return 1;
    }

    unsigned IntersectionTests::boxAndHalfSpace(
        const Box& box,
        const Physics::Plane& plane,
        std::vector<Contact*>& contacts
    )
    {
        Vector3 vertices[8];


        box.generateVetices(vertices);
        unsigned contactsCount = 0;
        for (int i = 0; i < 8; i++) {
            Vector3& vertexPos = vertices[i];
            float vertexDistance = dot(vertexPos, plane.normal);
            if (vertexDistance <= plane.offset + tolerance)
            {
                Contact* contact = new Contact();
                contact->contactPoint = plane.normal;
                contact->contactPoint *= (-vertexDistance + plane.offset) / 2; 
                contact->contactPoint += vertexPos;
                contact->contactNormal = plane.normal;
                contact->penetration = plane.offset - vertexDistance;
                contact->body1 = box.body;
                contact->body2 = plane.body;
                contacts.push_back(contact);
                contactsCount++;

            }

        }
        return contactsCount;

    }

    unsigned IntersectionTests::boxAndSphere(
        const Box& box,
        const Sphere& sphere,
        std::vector<Contact*>& contacts
    )
    {
        Vector3 center = sphere.getAxis(3);
        Vector3 relCenter = transformPoint(box.getTransform().Inverse(), center);

        if (fabsf(relCenter.x) - sphere.radius > box.halfSize.x ||
            fabsf(relCenter.y) - sphere.radius > box.halfSize.y ||
            fabsf(relCenter.z) - sphere.radius > box.halfSize.z)
        {
            return 0;
        }

        Vector3 closestPt(0, 0, 0);
        float dist;
        dist = relCenter.x;
        if (dist > box.halfSize.x) dist = box.halfSize.x;
        if (dist < -box.halfSize.x) dist = -box.halfSize.x;
        closestPt.x = dist;
        dist = relCenter.y;
        if (dist > box.halfSize.y) dist = box.halfSize.y;
        if (dist < -box.halfSize.y) dist = -box.halfSize.y;
        closestPt.y = dist;
        dist = relCenter.z;
        if (dist > box.halfSize.z) dist = box.halfSize.z;
        if (dist < -box.halfSize.z) dist = -box.halfSize.z;
        closestPt.z = dist;
        // Check we�re in contact.
        dist = (closestPt - relCenter).LengthSquared();
        if (dist > sphere.radius * sphere.radius) return 0;

        Vector3 closestPtWorld = transformPoint(box.getTransform(), closestPt);

        Contact* contact = new Contact();
        contact->contactPoint = closestPtWorld;
        contact->contactNormal = (closestPtWorld - center);
        contact->contactNormal.Normalize();
        contact->penetration = sphere.radius - sqrt(dist);

        contact->body1 = box.body;
        contact->body2 = sphere.body;

        contacts.push_back(contact);
        return 1;
    }

    static inline float transformToAxis(
        const Box& box,
        const Vector3& axis
    )
    {
        return
            box.halfSize.x * fabsf(dot(axis, box.getAxis(0))) +
            box.halfSize.y * fabsf(dot(axis, box.getAxis(1))) +
            box.halfSize.z * fabsf(dot(axis, box.getAxis(2)));
    }

    static inline float penetrationOnAxis(
        const Box& one,
        const Box& two,
        const Vector3& axis,
        const Vector3& toCentre
    )
    {
        float oneProject = transformToAxis(one, axis);
        float twoProject = transformToAxis(two, axis);

        float distance = fabsf(dot(toCentre, axis));

        return oneProject + twoProject - distance;
    }

    static inline bool tryAxis(
        const Box& one,
        const Box& two,
        Vector3 axis,
        const Vector3& toCentre,
        unsigned index,

        float& smallestPenetration,
        unsigned& smallestCase
    )
    {
        if (axis.LengthSquared() < 0.0001) return true;
        axis.Normalize();

        float penetration = penetrationOnAxis(one, two, axis, toCentre);

        if (penetration < 0) return false;
        if (penetration < smallestPenetration) {
            smallestPenetration = penetration;
            smallestCase = index;
        }
        return true;
    }

    void fillPointFaceBoxBox(
        const Box& one,
        const Box& two,
        const Vector3& toCentre,
        std::vector<Contact*>& contacts,
        unsigned best,
        float pen
    )
    {
        Contact* contact = new Contact();

        Vector3 normal = one.getAxis(best);
        if (dot(one.getAxis(best), toCentre) > 0)
        {
            normal = normal * -1.0f;
        }

        Vector3 vertex = two.halfSize;
        float proj;
        proj = dot(two.getAxis(0), normal);
        if (proj == 0 && dot(two.getAxis(0), toCentre) > 0) vertex.x = -vertex.x;
        else if (proj < 0) vertex.x = -vertex.x;

        proj = dot(two.getAxis(1), normal);
        if (proj == 0 && dot(two.getAxis(1), toCentre) > 0) vertex.y = -vertex.y;
        else if (proj < 0) vertex.y = -vertex.y;

        proj = dot(two.getAxis(2), normal);
        if (proj == 0 && dot(two.getAxis(2), toCentre) > 0) vertex.z = -vertex.z;
        else if (proj < 0) vertex.z = -vertex.z;


        contact->contactNormal = normal;
        contact->penetration = pen;
        contact->contactPoint = transformPoint(two.getTransform(), vertex);
        contact->body1 = one.body;
        contact->body2 = two.body;
        contacts.push_back(contact);
    }

    static inline Vector3 contactPoint(
        const Vector3& pOne,
        const Vector3& dOne,
        float oneSize,
        const Vector3& pTwo,
        const Vector3& dTwo,
        float twoSize,
        bool useOne)
    {
        Vector3 toSt, cOne, cTwo;
        float dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
        float denom, mua, mub;

        smOne = dOne.LengthSquared();
        smTwo = dTwo.LengthSquared();
        dpOneTwo = dot(dTwo, dOne);

        toSt = pOne - pTwo;
        dpStaOne = dot(dOne, toSt);
        dpStaTwo = dot(dTwo, toSt);

        denom = smOne * smTwo - dpOneTwo * dpOneTwo;

        if (fabsf(denom) < 0.0001f) {
            return useOne ? pOne : pTwo;
        }

        mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
        mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

        if (mua > oneSize ||
            mua < -oneSize ||
            mub > twoSize ||
            mub < -twoSize)
        {
            return useOne ? pOne : pTwo;
        }
        else
        {
            cOne = pOne + dOne * mua;
            cTwo = pTwo + dTwo * mub;

            return cOne * 0.5 + cTwo * 0.5;
        }
    }

    unsigned IntersectionTests::boxAndBox(
        const Box& one,
        const Box& two,
        std::vector<Contact*>& contacts
    )
    {
        Vector3 toCentre = two.getAxis(3) - one.getAxis(3);

        float pen = (std::numeric_limits<float>::max)();
        unsigned best = (std::numeric_limits<unsigned>::max)();
        if (!tryAxis(one, two, one.getAxis(0), toCentre, 0, pen, best)) return 0;
        if (!tryAxis(one, two, one.getAxis(1), toCentre, 1, pen, best)) return 0;
        if (!tryAxis(one, two, one.getAxis(2), toCentre, 2, pen, best)) return 0;

        if (!tryAxis(one, two, two.getAxis(0), toCentre, 3, pen, best)) return 0;
        if (!tryAxis(one, two, two.getAxis(1), toCentre, 4, pen, best)) return 0;
        if (!tryAxis(one, two, two.getAxis(2), toCentre, 5, pen, best)) return 0;

        unsigned bestSingleAxis = best;

        if (!tryAxis(one, two, cross(one.getAxis(0), two.getAxis(0)), toCentre, 6, pen, best)) return 0;
        if (!tryAxis(one, two, cross(one.getAxis(0), two.getAxis(1)), toCentre, 7, pen, best)) return 0;
        if (!tryAxis(one, two, cross(one.getAxis(0), two.getAxis(2)), toCentre, 8, pen, best)) return 0;
        if (!tryAxis(one, two, cross(one.getAxis(1), two.getAxis(0)), toCentre, 9, pen, best)) return 0;
        if (!tryAxis(one, two, cross(one.getAxis(1), two.getAxis(1)), toCentre, 10, pen, best)) return 0;
        if (!tryAxis(one, two, cross(one.getAxis(1), two.getAxis(2)), toCentre, 11, pen, best)) return 0;
        if (!tryAxis(one, two, cross(one.getAxis(2), two.getAxis(0)), toCentre, 12, pen, best)) return 0;
        if (!tryAxis(one, two, cross(one.getAxis(2), two.getAxis(1)), toCentre, 13, pen, best)) return 0;
        if (!tryAxis(one, two, cross(one.getAxis(2), two.getAxis(2)), toCentre, 14, pen, best)) return 0;

        if (best == (std::numeric_limits<unsigned>::max)()) return 0;

        if (best < 3)
        {
            fillPointFaceBoxBox(one, two, toCentre, contacts, best, pen);
            return 1;
        }
        else if (best < 6)
        {
            fillPointFaceBoxBox(two, one, toCentre * -1.0f, contacts, best - 3, pen);
            return 1;
        }
        else
        {
            best -= 6;
            unsigned oneAxisIndex = best / 3;
            unsigned twoAxisIndex = best % 3;
            Vector3 oneAxis = one.getAxis(oneAxisIndex);
            Vector3 twoAxis = two.getAxis(twoAxisIndex);
            Vector3 axis = cross(oneAxis, twoAxis);
            axis.Normalize();

            if (dot(axis, toCentre) > 0) axis = axis * -1.0f;

            Vector3 ptOnOneEdge = one.halfSize;
            Vector3 ptOnTwoEdge = two.halfSize;
            if (dot(one.getAxis(0), axis) > 0) ptOnOneEdge.x = -ptOnOneEdge.x;
            if (dot(one.getAxis(1), axis) > 0) ptOnOneEdge.y = -ptOnOneEdge.y;
            if (dot(one.getAxis(2), axis) > 0) ptOnOneEdge.z = -ptOnOneEdge.z;

            switch (oneAxisIndex)
            {
            case 0:
                ptOnOneEdge.x = 0;
                break;
            case 1:
                ptOnOneEdge.y = 0;
                break;
            case 2:
                ptOnOneEdge.z = 0;
                break;
            }
            if (dot(two.getAxis(0), axis) < 0) ptOnTwoEdge.x = -ptOnTwoEdge.x;
            if (dot(two.getAxis(1), axis) < 0) ptOnTwoEdge.y = -ptOnTwoEdge.y;
            if (dot(two.getAxis(2), axis) < 0) ptOnTwoEdge.z = -ptOnTwoEdge.z;
            switch (twoAxisIndex)
            {
            case 0:
                ptOnTwoEdge.x = 0;
                break;
            case 1:
                ptOnTwoEdge.y = 0;
                break;
            case 2:
                ptOnTwoEdge.z = 0;
                break;
            }

            ptOnOneEdge = transformPoint(one.getTransform(), ptOnOneEdge);
            ptOnTwoEdge = transformPoint(two.getTransform(), ptOnTwoEdge);

            Vector3 vertex = contactPoint(
                ptOnOneEdge, oneAxis, oneAxisIndex == 0 ? one.halfSize.x : oneAxisIndex == 1 ? one.halfSize.y : one.halfSize.z,
                ptOnTwoEdge, twoAxis, twoAxisIndex == 0 ? two.halfSize.x : twoAxisIndex == 1 ? two.halfSize.y : two.halfSize.z,
                bestSingleAxis > 2
            );

            Contact* contact = new Contact();
            contact->penetration = pen;
            contact->contactNormal = axis;
            contact->contactPoint = vertex;
            contact->body1 = one.body;
            contact->body2 = two.body;
            contacts.push_back(contact);
            return 1;
        }
        return 0;
    }
}