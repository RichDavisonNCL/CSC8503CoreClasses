#pragma once
#include "Texture.h"
#include "Shader.h"
#include "Mesh.h"

#include "Buffer.h"

namespace NCL {
	using namespace NCL::Rendering;

	namespace CSC8503 {
		class Transform;
		using namespace Maths;

		class RenderObject
		{
		public:
			RenderObject(Transform* parentTransform, Mesh* mesh, Texture* tex, Shader* shader) {
				buffer = nullptr;

				this->transform = parentTransform;
				this->mesh = mesh;
				this->texture = tex;
				this->shader = shader;
				this->colour = Vector4(1.0f, 1.0f, 1.0f, 1.0f);
			}
			~RenderObject() {}

			void SetDefaultTexture(Texture* t) {
				texture = t;
			}

			Buffer* GetGPUBuffer() const {
				return buffer;
			}

			void SetGPUBuffer(Buffer* b) {
				buffer = b;
			}

			Texture* GetDefaultTexture() const {
				return texture;
			}

			Mesh*	GetMesh() const {
				return mesh;
			}

			Transform*		GetTransform() const {
				return transform;
			}

			Shader*		GetShader() const {
				return shader;
			}

			void SetColour(const Vector4& c) {
				colour = c;
			}

			Vector4 GetColour() const {
				return colour;
			}

		protected:
			Buffer*			buffer;
			Mesh*			mesh;
			Texture*		texture;
			Shader*			shader;
			Transform*		transform;
			Vector4			colour;
		};
	}
}
