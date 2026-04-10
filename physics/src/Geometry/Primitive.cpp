#include "Geometry/Primitive.h"
#include "Collision/Contact.h"

namespace Physics {
	void Primitive::calculateInternals()
	{
		transform = body == nullptr ? offset : offset * body->getTransform();
	}
	Matrix Primitive::getTransform() const
	{
		return transform;
	}
	Primitive::Primitive(Matrix offset)
	{
		this->offset = offset;
	}
	Primitive::Primitive()
	{
	
	}
	Vector3 Primitive::getAxis(unsigned index) const
	{
		switch (index)
		{
		case 0:
			return transform.GetColumn(0).xyz();
		case 1:
			return transform.GetColumn(1).xyz();
		case 2:
			return transform.GetColumn(2).xyz();
		case 3:
			return transform.TranslationVector3D();
		}
		return Vector3(0.0f);
	}

}