#include "GLTFRenderObject.h"

using namespace NCL::CSC8503;
using namespace NCL;

GameTechMaterial dummyMat; //lmao

GLTFRenderObject::GLTFRenderObject(Transform& parentTransform, GLTFScene& scene)
	: RenderObject(parentTransform, nullptr, dummyMat)
{
	this->scene = &scene;
}