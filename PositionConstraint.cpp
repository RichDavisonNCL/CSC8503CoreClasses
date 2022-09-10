#include "PositionConstraint.h"
//#include "../../Common/Vector3.h"
#include "GameObject.h"
//#include "Debug.h"



using namespace NCL;
using namespace Maths;
using namespace CSC8503;

PositionConstraint::PositionConstraint(GameObject* a, GameObject* b, float d)
{
	objectA		= a;
	objectB		= b;
	distance	= d;
}

PositionConstraint::~PositionConstraint()
{

}

//a simple constraint that stops objects from being more than <distance> away
//from each other...this would be all we need to simulate a rope, or a ragdoll
void PositionConstraint::UpdateConstraint(float dt)	{
	Vector3 relativePos =	objectA->GetTransform().GetPosition() - 
							objectB->GetTransform().GetPosition();

	PhysicsObject* physA = objectA->GetPhysicsObject();
	PhysicsObject* physB = objectB->GetPhysicsObject();

	float currentDistance = relativePos.Length();

	float offset = distance - currentDistance;

	if (abs(offset) > 0.0f) {
		Vector3 offsetDir = relativePos.Normalised();

		Vector3 relativeVelocity = physA->GetLinearVelocity() - physB->GetLinearVelocity();

		float constraintMass = physA->GetInverseMass() + physB->GetInverseMass();


		if (constraintMass > 0.0f) {
			float velocityDot = Vector3::Dot(relativeVelocity, offsetDir); //how much of their relative force is affecting the constraint

			float bDefault = 0.08f;
			float b = -(bDefault / dt) * offset;

			//b = offset * dt;

			float lambda = -(velocityDot + b) / constraintMass;

			//if (abs(lambda) > 0.05f) {
			//	Debug::DrawLine(objectA->GetTransform().GetPosition(), objectA->GetTransform().GetPosition() - (offsetDir * distance), Vector4(1, 0, 0, 1));
			//}
			//else {
			//	Debug::DrawLine(objectA->GetTransform().GetPosition(), objectA->GetTransform().GetPosition() - (offsetDir * distance), Vector4(0, 1, 1, 1));
			//}

			//our 'jacobian' matrix would have 2 entries, moving the objects apart,
			//such that they satisfy the distance constraint. We can just model that
			//as 2 vectors in this case, with jn being our solution for lambda

			Vector3 aImpulse =  offsetDir * lambda;
			Vector3 bImpulse = -offsetDir * lambda;

			physA->ApplyLinearImpulse(aImpulse);
			physB->ApplyLinearImpulse(bImpulse);
		}
	}
}
