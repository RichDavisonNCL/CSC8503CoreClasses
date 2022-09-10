#pragma once
#include "../../Common/Matrix4.h"
#include "../../Common/Matrix3.h"
#include "../../Common/Vector3.h"
#include "../../Common/Quaternion.h"

#include <vector>

using std::vector;

using namespace NCL::Maths;

namespace NCL {
	namespace CSC8503 {
		class Transform
		{
		public:
			Transform();
			~Transform();

			Transform& SetPosition(const Vector3& worldPos);
			Transform& SetScale(const Vector3& worldScale);
			Transform& SetOrientation(const Quaternion& newOr);

			Vector3 GetPosition() const {
				return position;
			}

			Vector3 GetScale() const {
				return scale;
			}

			Quaternion GetOrientation() const {
				return orientation;
			}

			Matrix4 GetMatrix() const {
				return matrix;
			}

			void SetParentObject(Transform* t) {
				parentObject = t;
			}

			Transform& SetLocalPosition(const Vector3& localPos);
			Transform& SetLocalScale(const Vector3& localScale);
			Transform& SetLocalOrientation(const Quaternion& newOr);

		protected:
			void UpdateMatrix();
			void UpdateWorldValues();
			void UpdateLocalValues();

			Matrix4		matrix;

			Vector3		position;
			Vector3		scale;
			Quaternion	orientation;

			Vector3		localPosition;
			Vector3		localScale;
			Quaternion	localOrientation;

			Transform* parentObject;
		};
	}
}

