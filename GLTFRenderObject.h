#pragma once
#include "RenderObject.h"
#include "../GLTFLoader/GLTFLoader.h"
//OH NO. 

namespace NCL {
	using namespace NCL::Rendering;

	namespace CSC8503 {
		class Transform;
		using namespace Maths;

		class GLTFRenderObject : public RenderObject
		{
		public:
			GLTFRenderObject(Transform& parentTransform, GLTFScene& scene);
			~GLTFRenderObject() {}


			GLTFScene* GetScene() const {
				return scene;
			}

		protected:
			GLTFScene*			scene;
		};
	}
}
