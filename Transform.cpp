#include "Transform.h"

using namespace NCL::CSC8503;

Transform::Transform()
{
	scale			= Vector3(1, 1, 1);
	parentObject	= nullptr;
}

Transform::~Transform()
{

}

void Transform::UpdateMatrix() {
	matrix =
		Matrix4::Translation(position) *
		Matrix4(orientation) *
		Matrix4::Scale(scale);
}

Transform& Transform::SetPosition(const Vector3& worldPos) {
	position = worldPos;
	UpdateLocalValues();
	UpdateMatrix();
	return *this;
}

Transform& Transform::SetScale(const Vector3& worldScale) {
	scale = worldScale;
	UpdateLocalValues();
	UpdateMatrix();
	return *this;
}

Transform& Transform::SetOrientation(const Quaternion& worldOrientation) {
	orientation = worldOrientation;
	UpdateLocalValues();
	UpdateMatrix();
	return *this;
}

Transform& Transform::SetLocalPosition(const Vector3& newLocalPos) {
	localPosition = newLocalPos;
	UpdateWorldValues();
	UpdateMatrix();
	return *this;
}

Transform& Transform::SetLocalScale(const Vector3& newLocalScale) {
	localScale = newLocalScale;
	UpdateWorldValues();
	UpdateMatrix();
	return *this;
}

Transform& Transform::SetLocalOrientation(const Quaternion& newLocalOrientation) {
	localOrientation = newLocalOrientation;
	UpdateWorldValues();
	UpdateMatrix();
	return *this;
}

void Transform::UpdateWorldValues() {
	//Calculate proper world values from their local coordinates
	if (!parentObject) {
		position	= localPosition;
		scale		= localScale;
		orientation = localOrientation;
	}
	else {
		Matrix4 m	= parentObject->GetMatrix();

		position	= m * localPosition;
		orientation = parentObject->orientation * localOrientation;
//???
		scale		= (parentObject->orientation * parentObject->scale) * localScale;
	}
}

void Transform::UpdateLocalValues() {
	//Given a new world state, what's the local state?
	if (!parentObject) {
		localPosition		= position;
		localScale			= scale;
		localOrientation	= orientation;
	}
	else {
		//the tricky bit!

	}
}