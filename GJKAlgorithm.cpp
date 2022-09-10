#include "GJKAlgorithm.h"
#include "Simplex.h"
#include "../../Common/Vector3.h"
#include "Transform.h"
#include "Debug.h"
using namespace NCL;
using namespace Maths;
using namespace CSC8503;

bool GJKAlgorithm::GJKInserectionOBB(const NCL::OBBVolume& volumeA, const Transform& worldTransformA,
	const NCL::OBBVolume& volumeB, const Transform& worldTransformB, CollisionDetection::CollisionInfo& collisionInfo, float expansion) {
	Matrix4 aInvOrientation = Matrix4(worldTransformA.GetOrientation().Conjugate());
	Matrix4 bInvOrientation = Matrix4(worldTransformB.GetOrientation().Conjugate());

	Simplex s;

	expansion = 0.05f;

	Vector3 dir = worldTransformB.GetPosition() - worldTransformA.GetPosition();
	dir.Normalise();	//Direction vector between objects is a good first direction...

	if (dir.Length() == 0.0f) { //emergency!
		//collisionInfo.AddContactPoint(worldTransformA.GetWorldPosition(), Vector3(0, 1, 0), expansion);
		return true;
	}

	s.Add(MinkowskiSupport(worldTransformA, worldTransformB, dir, expansion));

	dir = -s.GetVertex(0);

	const int MAX_GJK_ITERATIONS = 32;
	int i = 0;

	for (i = 0; i < MAX_GJK_ITERATIONS; ++i) {
		dir.Normalise();
		Simplex::SupportPoint newPoint = MinkowskiSupport(worldTransformA, worldTransformB, dir, expansion);

		float dot = Vector3::Dot(newPoint.pos, dir); //if this point didnt' cross the origin, we can't get any closer!

		if (dot < 0) {
			return false;
		}

		s.Add(newPoint);

		int currentSize = s.GetSize();

		if (currentSize == 2) { //pick a direction to get the third vertex from
			Vector3 line = s.GetVertex(1) - s.GetVertex(0);
			Vector3 temp = Vector3::Cross(-s.GetVertex(0), line);

			dir = Vector3::Cross(line, temp);
		}
		else if (currentSize == 3) {
			dir = CheckTriangleSimplex(s);
		}
		else if (currentSize == 4) {
			bool check = CheckTetrahedronSimplex(s);

			if (check) {	//we can reduce the simplex to a tri test and continue searching
				dir = CheckTriangleSimplex(s, false);
			}
			else {				//the origin was inside the tetrahedron!
				//while we know that the tetrahedron now contains the origin
				//We don't know the collision points in world space, only the distance between them...(expansion)

				std::cout << "simplex contains the origin!" << std::endl;

				Vector3 collisionOnA;
				Vector3 collisionOnB;

				Vector3 d = s.ClosestPoint(&collisionOnA, &collisionOnB);

				float temp = d.Length();

				if (temp > 1) {
					bool a = true;
				}

				if (temp > expansion * 3) {
					return false;
				}

				Vector3 n = (collisionOnA - collisionOnB);

				Vector3 normal = (collisionOnA - collisionOnB).Normalised();

				std::cout << "Collision normal:" << normal << std::endl;

				collisionInfo.point.normal = normal;
				collisionInfo.point.localA = collisionOnA - worldTransformA.GetPosition();
				collisionInfo.point.localB = collisionOnB - worldTransformB.GetPosition();
				collisionInfo.point.penetration = n.Length() - (expansion*2);
				collisionInfo.point.penetration = d.Length();// -(expansion * 2);
				return true;		
			}
		}
	}
	if (i >= MAX_GJK_ITERATIONS - 1) {
		std::cout << "ran out of gjk iterations!" << std::endl;
	}
	return false;
}

Maths::Simplex::SupportPoint GJKAlgorithm::MinkowskiSupport(const Transform& worldTransformA, const Transform& worldTransformB, const Vector3& dir, float expansion) {
	Maths::Simplex::SupportPoint point;

	point.onA  = OBBSupport(worldTransformA,  dir,  expansion);
	point.onB  = OBBSupport(worldTransformB, -dir,  expansion);

	point.realA = OBBSupport(worldTransformA,  dir);
	point.realB = OBBSupport(worldTransformB, -dir);

	point.pos = point.onA - point.onB;

	return point;
}

Vector3 GJKAlgorithm::OBBSupport(const Transform& worldTransformA, const Vector3& dir, float expansion) {
	Vector3 localDir = worldTransformA.GetOrientation().Conjugate() * dir;
	Vector3 vertex;
	vertex.x = localDir.x < 0 ? -0.5f : 0.5f;
	vertex.y = localDir.y < 0 ? -0.5f : 0.5f;
	vertex.z = localDir.z < 0 ? -0.5f : 0.5f;

	return (worldTransformA.GetMatrix() * vertex) + (dir * expansion);
}

Vector3 GJKAlgorithm::CheckTriangleSimplex(Simplex &s, bool checkAbove) {
	Vector3 ab = s.GetVertex(1) - s.GetVertex(0);
	Vector3 ac = s.GetVertex(2) - s.GetVertex(0);

	Vector3 normal = Vector3::Cross(ac, ab);

	Vector3 ao = -s.GetVertex(0);

	if (Vector3::Dot(Vector3::Cross(normal, ab), ao) > 0) {
		s.SetToLine(s.GetSupportPoint(0), s.GetSupportPoint(1));
		Vector3 temp = Vector3::Cross(ab, Vector3::Cross(ao, ab));
		return temp;
	}

	if (Vector3::Dot(Vector3::Cross(ac, normal), ao) > 0) {
		s.SetToLine(s.GetSupportPoint(0), s.GetSupportPoint(2));
		return Vector3::Cross(ac, Vector3::Cross(ao, ac));
	}

	if (checkAbove) {
		if (Vector3::Dot(normal, ao) > 0) { //is the origin above the tri...
			return normal;
		}
		s.SetToTri(s.GetSupportPoint(0), s.GetSupportPoint(2), s.GetSupportPoint(1));

		return -normal;
	}
	else {
		s.SetToTri(s.GetSupportPoint(0), s.GetSupportPoint(1), s.GetSupportPoint(2));
		return normal;
	}
}
//returns true if this has been reduced to a triangle case
bool GJKAlgorithm::CheckTetrahedronSimplex(Simplex &s) {
	Vector3 ab = s.GetVertex(1) - s.GetVertex(0);
	Vector3 ac = s.GetVertex(2) - s.GetVertex(0);

	Vector3 ao = -s.GetVertex(0);

	if (Vector3::Dot(Vector3::Cross(ab, ac), ao) > 0) {
		//set to tri a b c -> no change
		return true;
	}
	Vector3 ad = s.GetVertex(3) - s.GetVertex(0);

	if (Vector3::Dot(Vector3::Cross(ac, ad), ao) > 0) {
		s.SetToTri(s.GetSupportPoint(0), s.GetSupportPoint(2), s.GetSupportPoint(3));
		return true;
	}

	if (Vector3::Dot(Vector3::Cross(ad, ab), ao) > 0) {
		s.SetToTri(s.GetSupportPoint(0), s.GetSupportPoint(3), s.GetSupportPoint(1));
		return true;
	}
	return false;
}