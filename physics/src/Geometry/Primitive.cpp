#include "Primitive.h"
#include "..\Collision\Contact.h"

namespace Physics {
	void Primitive::calculateInternals()
	{
		transform = body == nullptr ? offset : offset * body->getTransform();
	}
	DirectX::SimpleMath::Matrix Primitive::getTransform() const
	{
		return transform;
	}
	Primitive::Primitive(DirectX::SimpleMath::Matrix offset)
	{
		this->offset = offset;
	}
	Primitive::Primitive()
	{
	
	}
	DirectX::SimpleMath::Vector3 Primitive::getAxis(unsigned index) const
	{
		switch (index)
		{
		case 0:
			return DirectX::SimpleMath::Vector3{ transform._11, transform._12, transform._13 };
			break;
		case 1:
			return DirectX::SimpleMath::Vector3{ transform._21, transform._22, transform._23 };
			break;
		case 2:
			return DirectX::SimpleMath::Vector3{ transform._31, transform._32, transform._33 };
			break;
		case 3:
			return DirectX::SimpleMath::Vector3{ transform._41, transform._42, transform._43 };
			break;
		}


	}

}