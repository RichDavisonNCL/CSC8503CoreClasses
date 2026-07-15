#pragma once
#include "Texture.h"
#include "Shader.h"
#include "Mesh.h"
#include "MeshAnimation.h"

#include "Buffer.h"

namespace NCL {
	using namespace NCL::Rendering;

	namespace CSC8503 {
		class Transform;
		using namespace Maths;

		enum class MaterialType {
			Opaque,
			Transparent,
			Effect
		};

		struct GameTechMaterial
		{
			MaterialType	type	= MaterialType::Opaque;
			Texture* diffuseTex		= nullptr;
			Texture* bumpTex		= nullptr;
		};

		class RenderObject
		{
		public:
			RenderObject(Transform& parentTransform, Mesh* inMesh, const GameTechMaterial& material);
			~RenderObject() {}

			Buffer* GetGPUBuffer() const {
				return buffer;
			}

			void SetGPUBuffer(Buffer* b) {
				buffer = b;
			}

			GameTechMaterial GetMaterial() const
			{
				return material;
			}

			Mesh*	GetMesh() const {
				return mesh;
			}

			Transform& GetTransform() const
			{
				return transform;
			}

			void SetColour(const Vector4& c) {
				colour = c;
			}

			Vector4 GetColour() const {
				return colour;
			}

			void SetAnimation(MeshAnimation& inAnim);

			void UpdateAnimation(float dt);

			std::vector<Matrix4>& GetSkeleton() {
				return skeleton;
			}

		protected:
			Transform&			transform;
			GameTechMaterial	material;
			Mesh*				mesh;
			Buffer*				buffer;
	
			Vector4				colour;
			MeshAnimation*		anim;

			std::vector<Matrix4> skeleton;
			float	animTime		= 0.0f;
			int currentAnimFrame	= 0;
		};
	}
}
