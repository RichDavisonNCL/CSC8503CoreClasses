#include "GLTFRenderObject.h"

using namespace NCL::CSC8503;
using namespace NCL;

GameTechMaterial dummyMat{
	.type = MaterialType::Effect
}; //lmao

GLTFRenderObject::GLTFRenderObject(Transform& parentTransform, GLTFScene& scene)
	: RenderObject(parentTransform, nullptr, dummyMat)
{
	this->scene = &scene;
}

void GLTFRenderObject::UpdateAnimation(float dt) {
	if (!anim || anim->GetFrameCount() == 0) {
		return;
	}
	animTime -= dt;

	if (animTime <= 0) {
		currentAnimFrame++;
		animTime += anim->GetFrameTime();
		currentAnimFrame = (currentAnimFrame++) % anim->GetFrameCount();

		std::vector<Matrix4>const& inverseBindPose = scene->meshes[0]->GetInverseBindPose();

		if (inverseBindPose.size() != anim->GetJointCount()) {
			//oh no
			return;
		}

		const auto joints = anim->GetWorldMatrices(currentAnimFrame);

		for (int i = 0; i < skeleton.size(); ++i) {
			skeleton[i] = joints[i] * inverseBindPose[i];
		}
	}

}