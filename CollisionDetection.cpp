#include "CollisionDetection.h"
#include "CollisionVolume.h"
#include "AABBVolume.h"
#include "OBBVolume.h"
#include "SphereVolume.h"
#include "GJKAlgorithm.h"
#include "../../Common/Vector2.h"
#include "../../Common/Window.h"
#include "../../Common/Maths.h"

#include <list>

#include "../CSC8503Common/Simplex.h"

#include "Debug.h"

using namespace NCL;

bool CollisionDetection::RayPlaneIntersection(const Ray&r, const Plane&p, RayCollision& collisions) {
	float ln = Vector3::Dot(p.GetNormal(), r.GetDirection());

	if (ln == 0.0f) {
		return false; //direction vectors are perpendicular!
	}
	
	Vector3 planePoint = p.GetPointOnPlane();

	Vector3 pointDir = planePoint - r.GetPosition();

	float d = Vector3::Dot(pointDir, p.GetNormal()) / ln;

	collisions.collidedAt = r.GetPosition() + (r.GetDirection() * d);

	return true;
}

bool CollisionDetection::RayIntersection(const Ray& r,GameObject& object, RayCollision& collision) {
	bool hasCollided = false;

	const Transform& worldTransform = object.GetTransform();
	const CollisionVolume* volume	= object.GetBoundingVolume();

	if (!volume) {
		return false;
	}

	switch (volume->type) {
		case VolumeType::AABB:		hasCollided = RayAABBIntersection(r, worldTransform, (const AABBVolume&)*volume	, collision); break;
		case VolumeType::OBB:		hasCollided = RayOBBIntersection(r, worldTransform, (const OBBVolume&)*volume	, collision); break;
		case VolumeType::Sphere:	hasCollided = RaySphereIntersection(r, worldTransform, (const SphereVolume&)*volume	, collision); break;

		case VolumeType::Capsule:	hasCollided = RayCapsuleIntersection(r, worldTransform, (const CapsuleVolume&)*volume, collision); break;
	}

	return hasCollided;
}

bool CollisionDetection::RayBoxIntersection(const Ray&r, const Vector3& boxPos, const Vector3& boxSize, RayCollision& collision) {
	Vector3 boxMin = boxPos - boxSize;
	Vector3 boxMax = boxPos + boxSize;

	Vector3 rayPos = r.GetPosition();
	Vector3 rayDir = r.GetDirection();

	Vector3 tVals(-1, -1, -1);

	for (int i = 0; i < 3; ++i) {
		if (rayDir[i] > 0) {
			//if we're going forward along an axis, test against the 'min' side of that box
			tVals[i] = (boxMin[i] - rayPos[i]) / rayDir[i];
		}
		else if(rayDir[i] < 0){
			//but if we're going backward along an axis, test against the max side instead
			tVals[i] = (boxMax[i] - rayPos[i]) / rayDir[i];
		}
	}
	float best = tVals.GetMaxElement();
	if (best < 0) {
		return false;
	}

	//The tVal with the largest value is the plane we actually intersected with
	//we can then travel along the ray by T to get the intersection point
	Vector3 intersection = rayPos + (rayDir * best);

	const float epsilon = 0.0001f;

	for (int i = 0; i < 3; ++i) {
		if (intersection[i]+ epsilon < boxMin[i] || intersection[i]- epsilon > boxMax[i]) {
			return false;
		}
	}

	collision.collidedAt	= intersection;
	collision.rayDistance	= best;
	return true;
}


//https://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
bool CollisionDetection::RayAABBIntersection(const Ray&r, const Transform& worldTransform, const AABBVolume& volume, RayCollision& collision) {
	Vector3	boxPos	= worldTransform.GetPosition();
	Vector3 boxSize = volume.GetHalfDimensions();
	return RayBoxIntersection(r, boxPos, boxSize, collision);
}


bool CollisionDetection::RayOBBIntersection(const Ray&r, const Transform& worldTransform, const OBBVolume& volume, RayCollision& collision) {
	Quaternion orientation	= worldTransform.GetOrientation();
	Vector3 position		= worldTransform.GetPosition();
	
	Matrix3 transform	 = Matrix3(orientation);
	Matrix3 invTransform = Matrix3(orientation.Conjugate());

	Vector3 localRayPos = r.GetPosition() - position;

	Ray tempRay(invTransform * localRayPos, invTransform * r.GetDirection());

	bool collided = RayBoxIntersection(tempRay, Vector3(), volume.GetHalfDimensions(), collision);

	if (collided) {
		collision.collidedAt = transform * collision.collidedAt + position;
	}
	return collided;
}

float InigoCapsule(Vector3 rayOrigin, Vector3 RayDirection, Vector3 pa, Vector3 pb, float radius) {
	Vector3  lineSegment = pb - pa;
	Vector3  relativeOrigin = rayOrigin - pa;
	float baba = Vector3::Dot(lineSegment, lineSegment);			//line segment length squared
	float bard = Vector3::Dot(lineSegment, RayDirection);			//how aligned is the ray to the line segment
	float baoa = Vector3::Dot(lineSegment, relativeOrigin);
	float rdoa = Vector3::Dot(RayDirection, relativeOrigin);
	float oaoa = Vector3::Dot(relativeOrigin, relativeOrigin);
	float a = baba - bard * bard; //seems to end up at the length of the line segment when aligned with the origin, decreases when above/below
	float b = baba * rdoa - baoa * bard; //related to ray distance to the 'spindle', can reach zero if you are directly above/below
	float c = baba * oaoa - baoa * baoa - radius * radius * baba; //related directly to the ray ORIGIN distance to spindle - doesn't move with angle of ray, only of position
	float h = b * b - a * c;		//hypoteneus, so we're dealing with triangles//Gets smaller at the sides of the cylinder?//max value increases when not above/below


	if (h >= 0.0)
	{
		float t = (-b - sqrt(h)) / a; 
		float y = baoa + t * bard;	//How far 'up' from the base is the intersection point. Increases up the capsule's 'up' axis
		//std::cout << "a: " << a << "\n";
		//std::cout << "sqrt a: " << sqrt(a) << "\n";
		// body
		if (y > 0.0 && y < baba) return t;
		// caps
		Vector3 oc = (y <= 0.0) ? relativeOrigin : rayOrigin - pb;
		b = Vector3::Dot(RayDirection, oc);
		c = Vector3::Dot(oc, oc) - radius * radius;
		h = b * b - c;
		if (h > 0.0) return -b - sqrt(h);
	}
	return -1.0;
}

Vector3 MultiPlaneCapsuleIntersection(Vector3 rayOrigin, Vector3 rayDirection, Vector3 pa, Vector3 pb, float radius) {
	//We've got the line segment that forms the capsule
	//We've got the ray origin
	//The cross of these will be off to the side

	Vector3 sideDir = Vector3::Cross((pb - pa), (rayOrigin - pa));
	sideDir.Normalise();

	Plane centrePlane = Plane::PlaneFromTri(pa, pb, pb + sideDir);

	Vector3 up = (pb - pa).Normalised();

	float t = -(Vector3::Dot(rayOrigin, centrePlane.GetNormal()) + centrePlane.GetDistance()) / Vector3::Dot(rayDirection, centrePlane.GetNormal());

	//This is the point of intersection on the plane
	Vector3 planePoint = rayOrigin + (rayDirection * abs(t));

	Vector3 surfacePoint;
	Vector3 centrePoint;
	{//First, the plane normal
		float dist = Vector3::Dot((planePoint - pa) ,  up);
		centrePoint = pa + (up * dist); //projection of the impact point up the spindle

		float d = (centrePoint - planePoint).Length(); //how far away was the impact point
		//D is now one leg of a triangle
		//Hypoteneus MUST be of radius r
		float l = (radius * radius) - (d * d);

		surfacePoint = planePoint - (centrePlane.GetNormal() * sqrt(l));

		std::cout << "D: " << d << "\n";

		//return surfacePoint;
	}

	Vector3 pointA = surfacePoint;
	Vector3 pointB = surfacePoint + up;
	Vector3 pointC = surfacePoint + Vector3::Cross((surfacePoint + up).Normalised(), (surfacePoint - centrePoint).Normalised());

	Debug::DrawLine(surfacePoint, surfacePoint + (up * 10), Debug::GREEN, 10.0f);

	Debug::DrawLine(pointA, pointB, Debug::RED, 10.0f);
	Debug::DrawLine(pointB, pointC, Debug::RED, 10.0f);
	Debug::DrawLine(pointC, pointA, Debug::RED, 10.0f);

	//Debug::DrawLine(impactPoint - Vector3(0, 0, s), impactPoint + Vector3(0, 0, s), Debug::BLUE, 10.0f);



	//I can now make a plane at the 'cyclinder' edge with the correct normal
	Plane surfacePlane = Plane::PlaneFromTri(pointA, pointB, pointC);
	//Now we can ray intersect the surface plane!
	float st = -(Vector3::Dot(rayOrigin, surfacePlane.GetNormal()) + surfacePlane.GetDistance()) / Vector3::Dot(rayDirection, surfacePlane.GetNormal());

	Vector3 impactPoint = rayOrigin + (rayDirection * st);

	return impactPoint;
}

//TODO - we can reformulate this to act more like sphere intersection...
bool CollisionDetection::RayCapsuleIntersection(const Ray& r, const Transform& worldTransform, const CapsuleVolume& volume, RayCollision& collision) {

	Vector3 rayPos = r.GetPosition();
	Vector3 rayDir = r.GetDirection();

	Vector3 capsuleUp	= worldTransform.GetOrientation() * Vector3(0, 1, 0);
	float capsuleRadius = volume.GetRadius();

	Vector3 halfLine = capsuleUp * (volume.GetHalfHeight() - capsuleRadius);
	
	Vector3 pointA = worldTransform.GetPosition() - halfLine;
	Vector3 pointB = worldTransform.GetPosition() + halfLine;

	float distance = InigoCapsule(rayPos, rayDir, pointA, pointB, capsuleRadius);

	if (distance < 0) {
		return false;
	}

	Vector3 ingoImpactPoint = rayPos + (rayDir * distance);

	collision.collidedAt = ingoImpactPoint;
	collision.rayDistance = distance;

	float s = 0.1f;

	//Debug::DrawLine(impactPoint - Vector3(s, 0, 0), impactPoint + Vector3(s, 0, 0), Debug::BLUE, 10.0f);
	//Debug::DrawLine(impactPoint - Vector3(0, s, 0), impactPoint + Vector3(0, s, 0), Debug::BLUE, 10.0f);
	//Debug::DrawLine(impactPoint - Vector3(0, 0, s), impactPoint + Vector3(0, 0, s), Debug::BLUE, 10.0f);

	//return true;



//Way with 3 cases
	Vector3	capsulePos		= worldTransform.GetPosition();
	Vector3 up				= worldTransform.GetOrientation() * Vector3(0, 1, 0);
	Vector3	relativeRayPos  = r.GetPosition() - capsulePos;
	Vector3 dir				= relativeRayPos.Normalised();

	Vector3 right			= Vector3::Cross(dir, up).Normalised();
	float	radius			= volume.GetRadius();

	Plane centrePlane = Plane::PlaneFromTri(Vector3(), up, right);

	float t = -(Vector3::Dot(relativeRayPos, centrePlane.GetNormal()) + centrePlane.GetDistance()) / Vector3::Dot(r.GetDirection(), centrePlane.GetNormal());

	//This is the point of intersection on the plane
	Vector3 planePoint = relativeRayPos + (r.GetDirection() * t);

	float dot = Vector3::Dot(planePoint, up); //Project the point onto the capsule's 'spindle'

	Vector3 centrePoint = up * dot;

	Vector3 vSpherePos;

	if ((planePoint - centrePoint).Length() > radius) {
		//std::cout << "No Intersection!\n";
		//return false;
	}

	if (dot >= (volume.GetHalfHeight() - radius)) {
		//Ray collision against the top sphere!
		//std::cout << "Top case!\n";
		vSpherePos = up * (volume.GetHalfHeight() - radius);
	}
	else if (dot < -(volume.GetHalfHeight() - radius)) {
		//Ray Collision against the bottom sphere!
		//std::cout << "Bottom case!\n";
		vSpherePos = up * (volume.GetHalfHeight() - radius) * -1.0f;
	}
	else {
		//middle case
		//std::cout << "Middle case!\n";


		//Make it out of two offset vectors, one in direction of the plane normal, and one in direction of the 'shaft'

		float aAmount = 0.0f; //AMount to push back along the plane normal
		float bAmount = 0.0f; //Amount to push along the shaft lol

		{//First, the plane normal
			float d = (centrePoint - planePoint).Length(); //how far away was the impact point
			//D is now one leg of a triangle
			//Hypoteneus MUST be of radius r
			float l = (radius * radius) - (d * d);

			aAmount = sqrt(l); //If the ray was fully aligned with the normal, we'd have to move by this much
		}
		{//Now the shaft!
			//Another triangle, but this time all we know is an angle, and one leg, which must be R again

			float a = Vector3::Dot(r.GetDirection(), up);
			float sina = sin(acos(a));
			float hyp = radius / sina;

			float l = (hyp * hyp) - (radius * radius);
			bAmount = sqrt(l);
		}

		bAmount *= (aAmount / radius); //The more off to the side of the centre the plane impact point is, the further the ray can travel...
		//bAmount /= (aAmount / radius);


		//std::cout << "A: " << aAmount << " , B:" << bAmount << std::endl;


		Vector3 aOffset = centrePlane.GetNormal() * aAmount;
		Vector3 bOffset = up * bAmount;

		Vector3 finalOffset = aOffset + bOffset;

		float l = finalOffset.Length();

		Vector3 impactPoint = planePoint - r.GetDirection() * l;

		impactPoint += capsulePos;

		float broken = (impactPoint - ingoImpactPoint).Length();

		//std::cout << "Off by: " << broken << "\n";

		//float s = 0.1f;
		//Debug::DrawLine(impactPoint - Vector3(s, 0, 0), impactPoint + Vector3(s, 0, 0), Debug::BLUE, 10.0f);
		//Debug::DrawLine(impactPoint - Vector3(0, s, 0), impactPoint + Vector3(0, s, 0), Debug::BLUE, 10.0f);
		//Debug::DrawLine(impactPoint - Vector3(0, 0, s), impactPoint + Vector3(0, 0, s), Debug::BLUE, 10.0f);




		Vector3 planeTestPoint = MultiPlaneCapsuleIntersection(r.GetPosition(), r.GetDirection(), pointA, pointB, radius);

		float s = 0.1f;
		Debug::DrawLine(planeTestPoint - Vector3(s, 0, 0), planeTestPoint + Vector3(s, 0, 0), Debug::BLUE, 10.0f);
		Debug::DrawLine(planeTestPoint - Vector3(0, s, 0), planeTestPoint + Vector3(0, s, 0), Debug::BLUE, 10.0f);
		Debug::DrawLine(planeTestPoint - Vector3(0, 0, s), planeTestPoint + Vector3(0, 0, s), Debug::BLUE, 10.0f);


		float pd = (planeTestPoint - ingoImpactPoint).Length();
		std::cout << "OFF: " << pd << "\n";

		////Trig time!
		////We can work out an angle of a right angle triangle
		////between our 'up vector' and the vector to the ray 
		////we know one leg must be of length R
		////we can work out the hypoteneuse length, and travel along it to get to the real intersection point
		////

		//float a = Vector3::Dot(r.GetDirection(), up);

		//float sina = sin(acos(a));

		//float move = sina / radius;

		//move = a / radius;

		//std::cout << "move: " << move << "\n";




		//float typeADot = Vector3::Dot(r.GetDirection(), centrePlane.GetNormal());
		//float typeBDot = Vector3::Dot(r.GetDirection(), up);

		//std::cout << "A: " << typeADot << " B: " << typeBDot << "\n";

		//float d = (centrePoint - planePoint).Length();

		//float typeACalc = sqrt((radius * radius) - (d * d)); //how much to move back along the plane normal by


		//float typeBCalc = move;

		//typeBCalc = 0.0f;
		//typeACalc = 1.0f;


		//Vector3 typeAOffset = centrePlane.GetNormal() * typeACalc * typeADot * -1.0f;


		//Vector3 bDir = r.GetDirection();// Vector3::Cross(up, centrePlane.GetNormal());

		////bDir = centrePlane.GetNormal();


		//Vector3 typeBOffset = bDir * typeBCalc * typeBDot *-1.0f;

		//
		//Vector3 finalHitPoint = capsulePos + planePoint +typeAOffset + typeBOffset;

		//Debug::DrawLine(finalHitPoint, finalHitPoint + up * 10.0f, Vector4(1, 1, 1, 1), 5.0f);

		//return true;
	}

	Vector3 vDir		= vSpherePos.Normalised();
	float sphereProj	= Vector3::Dot(dir, r.GetDirection());

	//Vector3	relativeRayPos	= r.GetPosition() - capsulePos;

	return false;
}

bool CollisionDetection::RaySphereIntersection(const Ray&r, const Transform& worldTransform, const SphereVolume& volume, RayCollision& collision) {
	Vector3	spherePos		= worldTransform.GetPosition();
	float	sphereRadius	= volume.GetRadius();

	//Get the direction vector between the ray origin and the sphere origin
	Vector3 dir = (spherePos - r.GetPosition());

	//Then project the sphere's origin onto our ray direction vector

	float sphereProj = Vector3::Dot(dir, r.GetDirection());

	if (sphereProj < 0.0f) {
		return false;
	}

	//Now we know the projection point onto the direction vector
	//If it's further away than the radius of the sphere, it logically cannot be intersecting!
	//This is the point along the ray dir that the sphere direction projects on to.
	Vector3 point	= r.GetPosition() + (r.GetDirection() * sphereProj); 

	float sphereDist = (point - spherePos).Length();

	if (sphereDist > sphereRadius) {
		return false;
	}

	float offset = sqrt((sphereRadius * sphereRadius) - (sphereDist * sphereDist));

	collision.rayDistance	= sphereProj - offset;
	collision.collidedAt	= r.GetPosition() + (r.GetDirection() * collision.rayDistance);

	Debug::DrawLine(collision.collidedAt, collision.collidedAt + Vector3(0, 10, 0), Vector4(1, 0, 0, 1));

	return true;
}

Matrix4 GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
		Matrix4::Translation(position) *
		Matrix4::Rotation(-yaw, Vector3(0, -1, 0)) *
		Matrix4::Rotation(-pitch, Vector3(-1, 0, 0));

	return iview;
}

Matrix4 GenerateInverseProjection(float aspect, float nearPlane, float farPlane, float fov) {
	float negDepth = nearPlane - farPlane;

	float invNegDepth = negDepth / (2 * (farPlane * nearPlane));

	Matrix4 m;

	float h = 1.0f / tan(fov*PI_OVER_360);

	m.array[0] = aspect / h;
	m.array[5] = tan(fov*PI_OVER_360);
	m.array[10] = 0.0f;

	m.array[11] = invNegDepth;//// +PI_OVER_360;
	m.array[14] = -1.0f;
	m.array[15] = (0.5f / nearPlane) + (0.5f / farPlane);

	//Matrix4 temp = projection.Inverse();
	//return temp;
	return m;
}

Vector3 CollisionDetection::Unproject(const Vector3& screenPos, const Camera& cam) {
	Vector2 screenSize = Window::GetWindow()->GetScreenSize();

	float aspect	= screenSize.x / screenSize.y;
	float fov		= cam.GetFieldOfVision();
	float nearPlane = cam.GetNearPlane();
	float farPlane  = cam.GetFarPlane();

	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(cam) * GenerateInverseProjection(aspect, fov, nearPlane, farPlane);

	Matrix4 proj  = cam.BuildProjectionMatrix(aspect);

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(screenPos.x / (float)screenSize.x) * 2.0f - 1.0f,
		(screenPos.y / (float)screenSize.y) * 2.0f - 1.0f,
		(screenPos.z),
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}

Ray CollisionDetection::BuildRayFromMouse(const Camera& cam) {
	Vector2 screenMouse = Window::GetMouse()->GetAbsolutePosition();
	Vector2 screenSize	= Window::GetWindow()->GetScreenSize();

	//We remove the y axis mouse position from height as OpenGL is 'upside down',
	//and thinks the bottom left is the origin, instead of the top left!
	Vector3 nearPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		-0.99999f
	);

	//We also don't use exactly 1.0 (the normalised 'end' of the far plane) as this
	//causes the unproject function to go a bit weird. 
	Vector3 farPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		0.99999f
	);

	Vector3 a = Unproject(nearPos, cam);
	Vector3 b = Unproject(farPos, cam);
	Vector3 c = b - a;

	c.Normalise();

	//std::cout << "Ray Direction:" << c << std::endl;

	return Ray(cam.GetPosition(), c);
}

//http://bookofhook.com/mousepick.pdf
Matrix4 CollisionDetection::GenerateInverseProjection(float aspect, float fov, float nearPlane, float farPlane) {
	Matrix4 m;

	float t = tan(fov*PI_OVER_360);

	float neg_depth = nearPlane - farPlane;

	const float h = 1.0f / t;

	float c = (farPlane + nearPlane) / neg_depth;
	float e = -1.0f;
	float d = 2.0f*(nearPlane*farPlane) / neg_depth;

	m.array[0]  = aspect / h;
	m.array[5]  = tan(fov*PI_OVER_360);

	m.array[10] = 0.0f;
	m.array[11] = 1.0f / d;

	m.array[14] = 1.0f / e;

	m.array[15] = -c / (d*e);

	return m;
}

/*
And here's how we generate an inverse view matrix. It's pretty much
an exact inversion of the BuildViewMatrix function of the Camera class!
*/
Matrix4 CollisionDetection::GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
Matrix4::Translation(position) *
Matrix4::Rotation(yaw, Vector3(0, 1, 0)) *
Matrix4::Rotation(pitch, Vector3(1, 0, 0));

return iview;
}


/*
If you've read through the Deferred Rendering tutorial you should have a pretty
good idea what this function does. It takes a 2D position, such as the mouse
position, and 'unprojects' it, to generate a 3D world space position for it.

Just as we turn a world space position into a clip space position by multiplying
it by the model, view, and projection matrices, we can turn a clip space
position back to a 3D position by multiply it by the INVERSE of the
view projection matrix (the model matrix has already been assumed to have
'transformed' the 2D point). As has been mentioned a few times, inverting a
matrix is not a nice operation, either to understand or code. But! We can cheat
the inversion process again, just like we do when we create a view matrix using
the camera.

So, to form the inverted matrix, we need the aspect and fov used to create the
projection matrix of our scene, and the camera used to form the view matrix.

*/
Vector3	CollisionDetection::UnprojectScreenPosition(Vector3 position, float aspect, float fov, const Camera &c) {
	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(c) * GenerateInverseProjection(aspect, fov, c.GetNearPlane(), c.GetFarPlane());

	Vector2 screenSize = Window::GetWindow()->GetScreenSize();

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(position.x / (float)screenSize.x) * 2.0f - 1.0f,
		(position.y / (float)screenSize.y) * 2.0f - 1.0f,
		(position.z) - 1.0f,
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}



bool CollisionDetection::ObjectIntersection(GameObject* a, GameObject* b, CollisionInfo& collisionInfo) {
	const CollisionVolume* volA = a->GetBoundingVolume();
	const CollisionVolume* volB = b->GetBoundingVolume();

	if (!volA || !volB) {
		return false;
	}

	collisionInfo.a = a;
	collisionInfo.b = b;

	Transform& transformA = a->GetTransform();
	Transform& transformB = b->GetTransform();

	VolumeType pairType = (VolumeType)((int)volA->type | (int)volB->type);

	if (pairType == VolumeType::AABB) {
		return AABBIntersection((AABBVolume&)*volA, transformA, (AABBVolume&)*volB, transformB, collisionInfo);
	}

	if (pairType == VolumeType::Sphere) {
		return SphereIntersection((SphereVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}

	if (pairType == VolumeType::OBB) {
		//return GJKAlgorithm::GJKInserectionOBB((OBBVolume&)*volA, transformA, (OBBVolume&)*volB, transformB, collisionInfo);
		return OBBIntersectionSAT((OBBVolume&)*volA, transformA, (OBBVolume&)*volB, transformB, collisionInfo);
	}

	if (volA->type == VolumeType::AABB && volB->type == VolumeType::Sphere) {
		return AABBSphereIntersection((AABBVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBSphereIntersection((AABBVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	if (volA->type == VolumeType::OBB && volB->type == VolumeType::Sphere) {
			return OOBBSphereIntersection((OBBVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
		}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::OBB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return OOBBSphereIntersection((OBBVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}


	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::Sphere) {
		return SphereCapsuleIntersection((CapsuleVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::AABB) {
		return AABBCapsuleIntersection((CapsuleVolume&)*volA, transformA, (AABBVolume&)*volB, transformB, collisionInfo);
	}
	if (volB->type == VolumeType::Capsule && volA->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBCapsuleIntersection((CapsuleVolume&)*volB, transformB, (AABBVolume&)*volA, transformA, collisionInfo);
	}

	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::Capsule) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return SphereCapsuleIntersection((CapsuleVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	return false;
}

bool CollisionDetection::AABBTest(const Vector3& posA, const Vector3& posB, const Vector3& halfSizeA, const Vector3& halfSizeB) {
	Vector3 delta		= posB - posA;
	Vector3 totalSize	= halfSizeA + halfSizeB;

	if (abs(delta.x) < totalSize.x &&
		abs(delta.y) < totalSize.y &&
		abs(delta.z) < totalSize.z) {
		return true;
	}
	return false;
}

//AABB/AABB Collisions
bool CollisionDetection::AABBIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const AABBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 boxAPos = worldTransformA.GetPosition();
	Vector3 boxBPos = worldTransformB.GetPosition();

	Vector3 boxASize	= volumeA.GetHalfDimensions();
	Vector3 boxBSize	= volumeB.GetHalfDimensions();

	bool overlap = AABBTest(boxAPos, boxBPos, boxASize, boxBSize);

	//if the difference in position is less than the size between them...collision!
	if (overlap) {
		static const Vector3 faces[6] =
		{
			Vector3(-1,  0,  0),	Vector3( 1,  0,  0), 
			Vector3( 0, -1,  0), 	Vector3( 0,  1,  0), 
			Vector3( 0,  0, -1), 	Vector3( 0,  0,  1), 
		};

		Vector3 maxA = boxAPos + boxASize;
		Vector3 minA = boxAPos - boxASize;

		Vector3 maxB = boxBPos + boxBSize;
		Vector3 minB = boxBPos - boxBSize;

		float distances[6] =
		{
			(maxB.x - minA.x), // distance of box 'b' to face on 'left' side of 'a'.
			(maxA.x - minB.x), // distance of box 'b' to face on 'right' side of 'a'.
			(maxB.y - minA.y), // distance of box 'b' to face on 'bottom' side of 'a'.
			(maxA.y - minB.y), // distance of box 'b' to face on 'top' side of 'a'.
			(maxB.z - minA.z), // distance of box 'b' to face on 'far' side of 'a'.
			(maxA.z - minB.z)  // distance of box 'b' to face on 'near' side of 'a'.
		};
		
		float penetration = FLT_MAX;
		Vector3 axis;

		for (int i = 0; i < 6; i++)
		{
			if (distances[i] < penetration) {
				penetration = distances[i];
				axis		= faces[i];
			}
		}

		Vector3 localA = Vector3();
		Vector3 localB = Vector3();

		Debug::DrawLine(boxAPos + localA, boxBPos + localB, Vector4(1, 0, 0, 1));
		collisionInfo.AddContactPoint(localA, localB, axis, penetration);

		return true;
	}

	return false;
}
//Sphere / Sphere Collision

bool CollisionDetection::SphereIntersection(const SphereVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	float radii		= volumeA.GetRadius() + volumeB.GetRadius();
	Vector3 delta	= worldTransformB.GetPosition() - worldTransformA.GetPosition();

	float deltaLength = delta.Length();

	if (deltaLength < radii) {
		float	penetration		= (radii - deltaLength);
		Vector3 normal			= delta.Normalised();

		Vector3 localA =  normal * volumeA.GetRadius();
		Vector3 localB = -normal * volumeB.GetRadius();

		Debug::DrawLine(localA + worldTransformA.GetPosition(), localA + worldTransformA.GetPosition() + Vector3(0, 5, 0), Vector4(0, 1, 1, 1), 5);

		collisionInfo.AddContactPoint(localA, localB, normal, penetration);
		return true;//we're colliding!
	}
	return false;
}

//AABB - Sphere Collision
bool CollisionDetection::AABBSphereIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	Vector3 boxSize = volumeA.GetHalfDimensions();
	//local position of the sphere to the AABB
	Vector3 delta	= worldTransformB.GetPosition() - worldTransformA.GetPosition();

	Vector3 closestPointOnBox = Maths::Clamp(delta, -boxSize, boxSize);

	Vector3 localPoint = delta - closestPointOnBox;//How far away is the closest point in the box?
	float distance = (localPoint).Length();

	if (distance < volumeB.GetRadius()) {//yes, we're colliding!	
		Vector3 collisionNormal = localPoint.Normalised();
		float	penetration		= (volumeB.GetRadius() - distance);

		Vector3 localA		= Vector3();
		Vector3 localB		= -collisionNormal * volumeB.GetRadius();

		//if (distance < 0.00001f) {	//The sphere is INSIDE the box!
		//	collisionNormal = delta.Normalised();
		//	penetration = volumeB.GetRadius() + Vector3::Dot(collisionNormal, boxSize-delta);
		//}

		collisionInfo.AddContactPoint(localA, localB, collisionNormal, penetration);

		Debug::DrawLine(worldTransformA.GetPosition() + localA, worldTransformB.GetPosition() + localB, Vector4(1,1,0,1));
		return true;
	}
	return false;
}

float	bestP		= -FLT_MAX;
Vector3 bestAxis;
Vector3 bestPointA;
Vector3 bestPointB;

bool	flipState	= false;
bool edgeState = false;
Vector3 OBBSupport(const Transform& worldTransform, Vector3 worldDir) {
	Vector3 localDir = worldTransform.GetOrientation().Conjugate() * worldDir;
	Vector3 vertex;
	vertex.x = localDir.x < 0 ? -0.5f : 0.5f;
	vertex.y = localDir.y < 0 ? -0.5f : 0.5f;
	vertex.z = localDir.z < 0 ? -0.5f : 0.5f;

	return worldTransform.GetMatrix() * vertex;
}

//Vector3 OBBSupport(const Transform& worldTransform, Vector3 worldDir) {
//	Vector3 localDir = worldTransform.GetOrientation().Conjugate() * worldDir;
//	Vector3 vertex;
//	vertex.x = localDir.x < 0 ? -1.0f : 1.0f;
//	vertex.y = localDir.y < 0 ? -1.0f : 1.0f;
//	vertex.z = localDir.z < 0 ? -1.0f : 1.0f;
//
//	return worldTransform.GetMatrix() * vertex;
//}

//float CollisionArea(const Transform& worldTransform, Vector3 worldDir) {
//	Vector3 localDir = worldTransform.GetOrientation().Conjugate() * worldDir;
//	float max = abs(localDir.x);
//	Vector3 testDir = Vector3(0, 1, 1);
//
//	if (abs(localDir.y) > max) {
//		testDir = Vector3(1, 0, 1);
//	}
//	if (abs(localDir.z) > max) {
//		testDir = Vector3(1, 1, 0);
//	}
//
//	return Vector3::Dot(worldTransform.GetScale(),testDir);
//}

bool SATAxisTest(const Vector3& baseAxis , const Transform& worldTransformA, const Transform& worldTransformB, const OBBVolume& volumeA, const OBBVolume& volumeB, bool isEdge = false) {
	if (baseAxis.Length() < 0.5f) { //avoid cross issues!
		return true;
	}

	Vector3   axisNormal = baseAxis;

	const Transform* tA = &worldTransformA;
	const Transform* tB = &worldTransformB;
	const OBBVolume* vA = &volumeA;
	const OBBVolume* vB = &volumeB;

	bool flipped = false;

	//float rx = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
	//float ry = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
	//float rz = ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;

	//axisNormal.x += rx * 0.0001f;
	//axisNormal.y += ry * 0.0001f;
	//axisNormal.z += rz * 0.0001f;

	//axisNormal.Normalise();

	//float aArea = CollisionArea(worldTransformA, axisNormal);
	//float bArea = CollisionArea(worldTransformB, axisNormal);

	//if (aArea > bArea) {
	//	flipped = true;
	//	tA = &worldTransformB;
	//	tB = &worldTransformA;
	//	vA = &volumeB;
	//	vB = &volumeA;
	//}

	Vector3 minA = OBBSupport(*tA, -axisNormal);
	Vector3 maxA = OBBSupport(*tA,  axisNormal);

	Vector3 minB = OBBSupport(*tB, -axisNormal);
	Vector3 maxB = OBBSupport(*tB, axisNormal);

	float A = Vector3::Dot(minA, axisNormal);
	float B = Vector3::Dot(maxA, axisNormal);
	float C = Vector3::Dot(minB, axisNormal);
	float D = Vector3::Dot(maxB, axisNormal);

	//if ((B > C) && (B < D))

	if (A <= C && B >= C) {		
		float dist = B - C;
		if (dist < bestP) {
			bestP = dist;
			bestAxis = axisNormal;
			bestPointA = maxA;// -(bestAxis * bestP);
			bestPointB = minB;// +(bestAxis * bestP);

			//bestPoint = maxA - (bestAxis * bestP);
			flipState = flipped;
			edgeState = isEdge;
		}
		return true;
	}
	else if (C <= A && D >= A) {		
		float dist = D - A;
		if (dist < bestP) {
			bestP = dist;
			bestAxis = -axisNormal;

			bestPointA = minA;// +(bestAxis * bestP);
			bestPointB = maxB;// -(bestAxis * bestP);

			//bestPoint = minA + (bestAxis * bestP);
			flipState = flipped;
			edgeState = isEdge;
		}
		return true;
	}
	return false;
}


//https://www.programcreek.com/java-api-examples/index.php?sourzce_dir=react-master/src/main/java/com/flowpowered/react/collision/narrowphase/GJK/GJKAlgorithm.java
//
//http://www.dtecta.com/files/GDC2012_vandenBergen_Gino_Physics_Tut.pdf
//
//http://box2d.org/files/GDC2015/DirkGregorius_Contacts.pdf
bool CollisionDetection::OBBIntersectionSAT(
	const OBBVolume& volumeA, const Transform& worldTransformA,
	const OBBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {


	//Quaternion orientationA = worldTransformA.GetOrientation();
	//Quaternion orientationB = worldTransformB.GetOrientation();

	//Vector3 boxASize = volumeA.GetHalfDimensions();
	//Vector3 boxBSize = volumeB.GetHalfDimensions();

	//static const Vector3 faces[3] =
	//{
	//	Vector3(1, 0, 0),
	//	Vector3(0, 1, 0),
	//	Vector3(0, 0, 1)
	//};

	//Vector3 Axis[15];

	//for (int i = 0; i < 3; i++)
	//{
	//	Axis[i] = orientationA * faces[i];
	//	Axis[i].Normalise();
	//}

	//for (int i = 3; i < 6; i++)
	//{
	//	Axis[i] = orientationB * faces[i - 3];
	//	Axis[i].Normalise();
	//}

	//Axis[6] = Vector3::Cross(Axis[0], Axis[3]).Normalised();
	//Axis[7] = Vector3::Cross(Axis[0], Axis[4]).Normalised();
	//Axis[8] = Vector3::Cross(Axis[0], Axis[5]).Normalised();

	//Axis[9] = Vector3::Cross(Axis[1], Axis[3]).Normalised();
	//Axis[10] = Vector3::Cross(Axis[1], Axis[4]).Normalised();
	//Axis[11] = Vector3::Cross(Axis[1], Axis[5]).Normalised();

	//Axis[12] = Vector3::Cross(Axis[2], Axis[3]).Normalised();
	//Axis[13] = Vector3::Cross(Axis[2], Axis[4]).Normalised();
	//Axis[14] = Vector3::Cross(Axis[2], Axis[5]).Normalised();

	//Vector3 overLapAxis;
	//float penetration = FLT_MAX;
	//Vector3 contactA;
	//Vector3 contactB;

	//for (int i = 0; i < 15; i++)
	//{
	//	Vector3 maxA = OBBSupport(worldTransformA, Axis[i]);
	//	Vector3 minA = OBBSupport(worldTransformA, -Axis[i]);

	//	Vector3 maxB = OBBSupport(worldTransformB, Axis[i]);
	//	Vector3 minB = OBBSupport(worldTransformB, -Axis[i]);

	//	maxA = maxA * boxASize;
	//	minA = minA * boxASize;

	//	maxB = maxB * boxBSize;
	//	minB = minB * boxBSize;

	//	//maxA = worldTransformA.GetOrientation() * maxA;
	//	//minA = worldTransformA.GetOrientation() * minA;

	//	//maxB = worldTransformB.GetOrientation() * maxB;
	//	//minB = worldTransformB.GetOrientation() * minB;

	//	float MaxExtentA = Vector3::Dot(Axis[i], maxA);
	//	float MinExtentA = Vector3::Dot(Axis[i], minA);
	//	float MaxExtentB = Vector3::Dot(Axis[i], maxB);
	//	float MinExtentB = Vector3::Dot(Axis[i], minB);


	//	if ((MinExtentA <= MinExtentB) && (MaxExtentA >= MinExtentB)
	//		|| (MinExtentB <= MinExtentA) && (MaxExtentB >= MinExtentA))
	//	{
	//		//test = true;
	//		//overlap = true;		
	//		float tempPenetration = 0;

	//		Vector3 axis = Axis[i];

	//		if ((MinExtentA < MinExtentB) && (MaxExtentA > MinExtentB))
	//		{
	//			tempPenetration = abs(MaxExtentA - MinExtentB);
	//			
	//		}

	//		if ((MinExtentB < MinExtentA) && (MaxExtentB > MinExtentA))
	//		{
	//			tempPenetration = abs(MinExtentA - MaxExtentB);
	//			axis = -axis;
	//		}

	//		if (tempPenetration != 0.0f && penetration > tempPenetration)
	//		{
	//			penetration = tempPenetration;
	//			overLapAxis = axis;

	//			//temp need to better think this  out

	//			//Vector3 maxAdist = boxAPos - maxA;
	//			//Vector3 minAdist = boxAPos - minA;

	//			//if (maxAdist.Length() > minAdist.Length())
	//			//{
	//			//	contactA = maxA;
	//			//}
	//			//else
	//			//{
	//			//	contactA = minA;
	//			//}

	//			//Vector3 maxBdist = boxBPos - maxB;
	//			//Vector3 minBdist = boxBPos - minB;

	//			//if (maxBdist.Length() > minBdist.Length())
	//			//{
	//			//	contactB =  maxB;
	//			//}
	//			//else
	//			//{
	//			//	contactB = minB;
	//			//}
	//		}
	//	}
	//	else
	//	{
	//		//axis has a seperation on it so the two volumes cannot be overalping
	//		return false;
	//	}
	//}

	//collisionInfo.AddContactPoint(Vector3(), Vector3(), overLapAxis, penetration);

	//return true;












	bestP		= FLT_MAX;
	flipState	= false;
	edgeState	= false;

	Quaternion	aOrientation	= worldTransformA.GetOrientation();
	Quaternion	bOrientation	= worldTransformB.GetOrientation();

	const Vector3 aNormals[3]{
		aOrientation * Vector3(1,0,0),
		aOrientation * Vector3(0,1,0),
		aOrientation * Vector3(0,0,1)
	};

	const Vector3 bNormals[3]{
		bOrientation * Vector3(1,0,0),
		bOrientation * Vector3(0,1,0),
		bOrientation * Vector3(0,0,1)
	};

	if (!SATAxisTest(aNormals[0], worldTransformA, worldTransformB, volumeA, volumeB) ||
		!SATAxisTest(aNormals[1], worldTransformA, worldTransformB, volumeA, volumeB) ||
		!SATAxisTest(aNormals[2], worldTransformA, worldTransformB, volumeA, volumeB) ||
		!SATAxisTest(bNormals[0], worldTransformA, worldTransformB, volumeA, volumeB) ||
		!SATAxisTest(bNormals[1], worldTransformA, worldTransformB, volumeA, volumeB) ||
		!SATAxisTest(bNormals[2], worldTransformA, worldTransformB, volumeA, volumeB) ||

		!SATAxisTest(Vector3::Cross(aNormals[0], bNormals[0]), worldTransformA, worldTransformB, volumeA, volumeB, true) ||
		!SATAxisTest(Vector3::Cross(aNormals[0], bNormals[1]), worldTransformA, worldTransformB, volumeA, volumeB, true) ||
		!SATAxisTest(Vector3::Cross(aNormals[0], bNormals[2]), worldTransformA, worldTransformB, volumeA, volumeB, true) ||
		!SATAxisTest(Vector3::Cross(aNormals[1], bNormals[0]), worldTransformA, worldTransformB, volumeA, volumeB, true) ||
		!SATAxisTest(Vector3::Cross(aNormals[1], bNormals[1]), worldTransformA, worldTransformB, volumeA, volumeB, true) ||
		!SATAxisTest(Vector3::Cross(aNormals[1], bNormals[2]), worldTransformA, worldTransformB, volumeA, volumeB, true) ||
		!SATAxisTest(Vector3::Cross(aNormals[2], bNormals[0]), worldTransformA, worldTransformB, volumeA, volumeB, true) ||
		!SATAxisTest(Vector3::Cross(aNormals[2], bNormals[1]), worldTransformA, worldTransformB, volumeA, volumeB, true) ||
		!SATAxisTest(Vector3::Cross(aNormals[2], bNormals[2]), worldTransformA, worldTransformB, volumeA, volumeB, true)
		) {
		return false;
	}

	Debug::DrawLine(bestPointA, bestPointB, Debug::CYAN, 10);

	float aa = (bestPointA - worldTransformA.GetPosition()).LengthSquared();
	float ab = (bestPointB - worldTransformA.GetPosition()).LengthSquared();

	float ba = (bestPointA - worldTransformB.GetPosition()).LengthSquared();
	float bb = (bestPointB - worldTransformB.GetPosition()).LengthSquared();


	Debug::DrawAxisLines(worldTransformA.GetMatrix(), 2.0f, 0.1f);
	Debug::DrawAxisLines(worldTransformB.GetMatrix(), 2.0f, 0.1f);

	Vector3 localA;// = (bestPoint - worldTransformA.GetPosition());
	Vector3 localB;// = (bestPoint - worldTransformB.GetPosition());

	if (aa < ab) {
		localA = (bestPointA - worldTransformA.GetPosition());
	}
	else {
		localA = (bestPointA - worldTransformA.GetPosition());
	}

	if (ba < bb) {
		localB = (bestPointA - worldTransformB.GetPosition());
	}
	else {
		localB = (bestPointB - worldTransformB.GetPosition());
	}



	//std::cout << -bestP << std::endl;

	if (flipState) {
		GameObject* temp = collisionInfo.a;
		collisionInfo.a = collisionInfo.b;
		collisionInfo.b = temp;
		collisionInfo.AddContactPoint(localB, localA, bestAxis, bestP);

		//collisionInfo.point.localA
	}
	else {
		collisionInfo.AddContactPoint(localA, localB, bestAxis, bestP);
	}

	if (edgeState) {
		std::cout << "Edge collision!\n";
	}
	else {
		std::cout << "Face collision!\n";
	}

	return true;
}


bool  CollisionDetection::OOBBSphereIntersection(const OBBVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Quaternion obbOreintation		= worldTransformA.GetOrientation();
	Quaternion obbInvOrientation	= obbOreintation.Conjugate();

	Vector3 obbPosition			= worldTransformA.GetPosition();

	Vector3 relativeSpherePos = worldTransformB.GetPosition() - obbPosition;
	Vector3 orientedSpherePos = obbInvOrientation * relativeSpherePos;

	Vector3 boxSize = volumeA.GetHalfDimensions();
	//local position of the sphere to the AABB
	Vector3 delta = orientedSpherePos;

	Vector3 closestPointOnBox = Maths::Clamp(delta, -boxSize, boxSize);

	Vector3 localPoint = delta - closestPointOnBox;//How far away is the closest point in the box?
	float distance = (localPoint).Length();


	if (distance < volumeB.GetRadius()) {//yes, we're colliding!	
		Vector3 collisionNormal = obbOreintation * localPoint.Normalised();
		float	penetration = (volumeB.GetRadius() - distance);

		Vector3 localA = obbOreintation * closestPointOnBox;
		Vector3 localB = -collisionNormal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNormal, penetration);

		Debug::DrawLine(worldTransformA.GetPosition() + localA, worldTransformB.GetPosition() + localB, Vector4(1, 1, 0, 1));
		return true;
	}
	return false;

}

bool CollisionDetection::AABBCapsuleIntersection(
	const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const AABBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 capsulePos			= worldTransformA.GetPosition();
	Vector3	relativeCapsulePos	= capsulePos - worldTransformB.GetPosition();
	Vector3	relativeBoxPos		= worldTransformA.GetOrientation().Conjugate() * -relativeCapsulePos;

	float	capsuleRadius	= volumeA.GetRadius();
	Vector3	boxSize			= volumeB.GetHalfDimensions();

	Vector3 up			= worldTransformA.GetOrientation() * Vector3(0, 1, 0);
	Vector3 topPos		= up * (volumeA.GetHalfHeight() - capsuleRadius);
	Vector3 bottomPos	= -topPos;

	float topDot	= Vector3::Dot(-up, relativeBoxPos - topPos);
	float bottomDot = Vector3::Dot( up, relativeBoxPos - bottomPos);

	Vector3 comparePoint;



	if (topDot < 0) {	//AABB is above the capsule
		comparePoint = topPos;
	}
	else if (bottomDot < 0) {//AABB is below the capsule
		comparePoint = bottomPos;
	}
	else { //Sphere is somewhat inline with the capsule
		float dot	 = Vector3::Dot(-up, (relativeCapsulePos));
		comparePoint = (up * dot);
	}

	Vector3 worldComparePoint = comparePoint + capsulePos;

	Vector3 closestPointOnBox = Maths::Clamp(comparePoint + relativeCapsulePos, -boxSize, boxSize);

	Vector3 worldPoint = closestPointOnBox + worldTransformB.GetPosition();

	Debug::DrawLine(worldComparePoint, worldPoint, Debug::CYAN);

	Vector3 localPoint = worldPoint - worldComparePoint;//How far away is the closest point in the box?
	float distance = (localPoint).Length();

	if (distance < capsuleRadius) {//yes, we're colliding!	
		Vector3 collisionNormal = localPoint.Normalised();
		float	penetration = (capsuleRadius - distance);

		Vector3 localA = worldPoint - capsulePos;
		Vector3 localB = Vector3();

		collisionInfo.AddContactPoint(localA, localB, collisionNormal, penetration);

		Debug::DrawLine(worldTransformA.GetPosition() + localA, worldTransformB.GetPosition() + localB, Vector4(1, 1, 0, 1));
		return true;
	}
	return false;

}


bool CollisionDetection::SphereCapsuleIntersection(
	const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {


	const Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();
	Vector3 Up = worldTransformA.GetOrientation() * Vector3(0, 1, 0);
	float D = Clamp(Vector3::Dot(delta, Up), -(volumeA.GetHalfHeight() - volumeA.GetRadius()), volumeA.GetHalfHeight() - volumeA.GetRadius());
	SphereVolume sphereVolume(volumeA.GetRadius());
	Transform transform;
	transform.SetPosition(worldTransformA.GetOrientation() * (worldTransformA.GetPosition() + Vector3(0, D, 0)));

	transform.SetPosition((worldTransformA.GetPosition() + (worldTransformA.GetOrientation() * Vector3(0, D, 0))));

	if (SphereIntersection(sphereVolume, transform, volumeB, worldTransformB, collisionInfo))
	{
		collisionInfo.point.localA = transform.GetPosition() - worldTransformA.GetPosition() + collisionInfo.point.localA;
		return true;
	}
	return false;







	//Vector3 capsulePos			= worldTransformA.GetPosition();
	//Vector3	relativeSpherePos	= worldTransformB.GetPosition() - capsulePos;

	//float	capsuleRadius = volumeA.GetRadius();
	//float	sphereRadius  = volumeB.GetRadius();

	//Vector3 up			= worldTransformA.GetOrientation() * Vector3(0, 1, 0);

	//Vector3 topPos		= up * (volumeA.GetHalfHeight() - capsuleRadius);
	//Vector3 bottomPos	= -topPos;

	//float topDot	= Vector3::Dot(-up, relativeSpherePos - topPos);
	//float bottomDot = Vector3::Dot( up, relativeSpherePos - bottomPos);

	//Vector3 comparePoint;

	//if (topDot < 0) {	//Sphere is above the capsule
	//	comparePoint = topPos;
	//}
	//else if (bottomDot < 0) {//Sphere is below the capsule
	//	comparePoint = bottomPos;
	//}
	//else { //Sphere is somewhat inline with the capsule
	//	float dot	 = Vector3::Dot(up, (relativeSpherePos));
	//	comparePoint = (up * dot);
	//}

	//Vector3 dir  = relativeSpherePos - comparePoint;

	//float distSqr	= dir.LengthSquared();
	//float radii		= sphereRadius + capsuleRadius;

	//if (distSqr >= radii * radii){
	//	return false;
	//}
	//float l = sqrt(distSqr);

	//float penetration = radii - l;

	//Vector3 normal = dir / l;

	//collisionInfo.AddContactPoint(comparePoint + (normal*capsuleRadius),normal * -sphereRadius , normal, penetration);

	//return true;
}