#pragma once

#define _USE_MATH_DEFINES

#include <mathfu/vector.h>
#include <mathfu/matrix.h>
#include <mathfu/quaternion.h>

namespace Physics
{
    using Scalar = float;
    using Vector3 = mathfu::Vector<Scalar, 3>;
    using Matrix = mathfu::Matrix<Scalar, 4, 4>;
    using Quaternion = mathfu::Quaternion<Scalar>;

    inline Vector3 transformPoint(const Matrix& m, const Vector3& v)
    {
        return (m * mathfu::Vector<Scalar, 4>(v, 1.0f)).xyz();
    }

    inline Vector3 transformVector(const Matrix& m, const Vector3& v)
    {
        return (m * mathfu::Vector<Scalar, 4>(v, 0.0f)).xyz();
    }

    inline Vector3 cross(const Vector3& a, const Vector3& b)
    {
        return Vector3::CrossProduct(a, b);
    }

    inline float dot(const Vector3& a, const Vector3& b)
    {
        return Vector3::DotProduct(a, b);
    }
}
