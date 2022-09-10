#include "OrientationConstraint.h"
#include "GameObject.h"

using namespace NCL;
using namespace Maths;
using namespace CSC8503;

OrientationConstraint::OrientationConstraint(GameObject* a, GameObject* b)
{
	objectA = a;
	objectB = b;
}

OrientationConstraint::~OrientationConstraint()
{

}

void OrientationConstraint::UpdateConstraint(float dt) {
	//Let's make it so their fwd axis must be aligned?

	PhysicsObject* physA = objectA->GetPhysicsObject();
	PhysicsObject* physB = objectB->GetPhysicsObject();

	Vector3 aPos = objectA->GetTransform().GetPosition();
	Vector3 bPos = objectB->GetTransform().GetPosition();

	Vector3 aFwd = objectA->GetTransform().GetOrientation() * Vector3(0, 0, -1);
	Vector3 bFwd = objectA->GetTransform().GetOrientation() * Vector3(0, 0, -1);

	Vector3 d = bPos - aPos;

	Vector3 n = d.Normalised();

	//object A should by directly facing n, B should be -n

	float ad = Vector3::Dot(n, aFwd);
	float bd = Vector3::Dot(-n, bFwd);

	Vector3 aAxis = Vector3::Cross(aFwd, n);
	Vector3 bAxis = Vector3::Cross(bFwd, n);

	Vector3 adc = Vector3::Cross(n.Normalised(), aFwd.Normalised());

	float adcl = adc.Length();

	if (ad < 1.0f) {
		Vector3 inertiaA = Vector3::Cross(physA->GetInertiaTensor() * aAxis, aFwd);
		float d = Vector3::Dot(inertiaA, aFwd);
		Vector3 aImpulse = -aAxis * ((1.0 - ad) * 5);

		if (d > 1.0f) {
			d = 1.0f;
		}
		if (d < -1.0f) {
			d = -1.0f;
		}

		aImpulse = -aAxis * (acos(d) * 50);

		if (aImpulse.x != aImpulse.x) {
			bool a = true;
		}


		Vector3 angVelocityA = Vector3::Cross(physA->GetAngularVelocity(), aImpulse);

		/*
		float l = Vector3::Dot(aImpulse, Vector3::Cross(aImpulse, physA->GetAngularVelocity()));
		aImpulse = aImpulse * l;
		//aImpulse = aAxis;		
		*/
		aImpulse = aImpulse - physA->GetAngularVelocity();

		//float newl = 1.0  -Vector3::Dot(aImpulse, physA->GetAngularVelocity());

		//aImpulse = aImpulse * newl;

		std::cout << aImpulse << "\n";

		physA->ApplyAngularImpulse(aImpulse);
	}
}