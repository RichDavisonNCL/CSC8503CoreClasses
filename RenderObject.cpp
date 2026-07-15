#include "RenderObject.h"
#include "Mesh.h"

using namespace NCL::CSC8503;
using namespace NCL;


RenderObject::RenderObject(Transform& parentTransform, Mesh* mesh, const GameTechMaterial& material)
	: transform(parentTransform)
{
	this->material	= material;
	this->mesh		= mesh;
	this->colour	= Vector4(1.0f, 1.0f, 1.0f, 1.0f);

	buffer	= nullptr;
	anim	= nullptr;
}

void RenderObject::SetAnimation(MeshAnimation& inAnim) {
	anim = &inAnim;

	skeleton.resize(anim->GetJointCount());
}

void RenderObject::UpdateAnimation(float dt) {
	if (!mesh || !anim || anim->GetFrameCount() == 0) {
		return;
	}
	animTime -= dt;

	if (animTime <= 0) {
		currentAnimFrame++;
		animTime += anim->GetFrameTime();
		currentAnimFrame = (currentAnimFrame++) % anim->GetFrameCount();

		std::vector<Matrix4>const&  inverseBindPose = mesh->GetInverseBindPose();

		if (inverseBindPose.size() != anim->GetJointCount()) {
			//oh no
			return;
		}

		const Matrix4* joints = anim->GetWorldMatrices(currentAnimFrame);

		for (int i = 0; i < skeleton.size(); ++i) {
			skeleton[i] = joints[i] * inverseBindPose[i];
		}
	}
}