#include "Simplex.h"
#include "Plane.h"
#include "Debug.h"
#include "Maths.h"
using namespace NCL::Maths;

Simplex::Simplex()
{
	index = 0;
	size  = 0;
}

Simplex::~Simplex()
{
}

void Simplex::SetToPoint(SupportPoint a) {
	verts[0] = a;
	index = 1;
	size  = 1;
}

void Simplex::SetToTri(SupportPoint a, SupportPoint b, SupportPoint c) {
	verts[0]	= a;
	verts[1]	= b;
	verts[2]	= c;
	index		= 3;
	size		= 3;
}

void Simplex::SetToLine(SupportPoint a, SupportPoint b) {
	verts[0] = a;
	verts[1] = b;
	index	= 2;
	size	= 2;
}

void Simplex::Add(SupportPoint a) {
	verts[index]	= a;
	index			= (index + 1) % 4;
	size++;
}

void Simplex::RemoveOldestSupportPoint() {
	size--;
}

//float Simplex::GetClosestDistance() {
//	float distance = GetVertex(0).Length();
//
//	for (int i = 1; i < size; ++i) {
//		float tempDist = GetVertex(i).Length();
//		distance = min(distance, tempDist);
//	}
//	return distance;
//}

//bool Simplex::ExtractCollisionInfo(Vector3& normal, float& penetration) {
//	if (size != 4) {
//		return false;
//	}
//	//determine tri with smallest projection to the origin
//
//	Plane bestPlane;
//
//	float bestDistance = FLT_MAX;
//
//	int bestIndices[3];
//
//	for (int i = 0; i < 4; ++i) {
//		int ia = i;
//		int ib = (i + 1) % 4; //getting the 'older' points on purpose
//		int ic = (i + 2) % 4;
//
//		Vector3 a = GetVertex(ia);
//		Vector3 b = GetVertex(ib);
//		Vector3 c = GetVertex(ic);
//
//		Plane p = Plane::PlaneFromTri(a, b, c);
//
//		float absDistance = abs(p.GetDistance());
//
//		if (absDistance < bestDistance) {
//			bestPlane = p;
//			bestDistance = absDistance;
//			bestIndices[0] = ia;
//			bestIndices[1] = ib;
//			bestIndices[2] = ic;
//		}
//	}
//	//we now have the closest tri to the origin
//
//	Vector3 planePoint = bestPlane.ProjectPointOntoPlane(Vector3());
//
//	Vector3 a = GetVertex(bestIndices[0]);
//	Vector3 b = GetVertex(bestIndices[1]);
//	Vector3 c = GetVertex(bestIndices[2]);
//
//	float triArea = abs(Maths::CrossAreaOfTri(a, b, c));
//
//	float alpha = abs(Maths::CrossAreaOfTri(b, c, planePoint));
//	float beta	= abs(Maths::CrossAreaOfTri(a, c, planePoint));
//	float gamma = abs(Maths::CrossAreaOfTri(a, b, planePoint));
//
//	float totalArea = 1.0f / triArea;
//
//	alpha	= alpha * totalArea;
//	beta	= beta * totalArea;
//	gamma	= gamma * totalArea;
//
//	float sum = alpha + beta + gamma;
//
//	if (sum > 1.0001f || sum < 0.9999) {
//		return false;
//	}
//	penetration = abs(bestPlane.GetDistance());
//	normal		= bestPlane.GetNormal();
//
//	return true;
//}

//http://allenchou.net/2013/12/game-physics-collision-detection-gjk/
//http://box2d.org/files/GDC2010/GDC2010_Catto_Erin_GJK.pdf
//https://stackoverflow.com/questions/9605556/how-to-project-a-point-onto-a-plane-in-3d 
//void Simplex::DetermineBestCollisions(Vector3& onA, Vector3& onB)	{
//	if (size != 4) {
//		return;
//	}
//	////there's 4 triangles in total in the simplex, which one is closest to the origin?
//	int bestIndex = 0;
//	float bestTri = FLT_MAX;
//
//	Vector3 bestPoint;
//
//	for (int i = 0; i < 4; ++i) {
//		int ia = i;
//		int ib = (i + 1) % 4; //getting the 'older' points on purpose
//		int ic = (i + 2) % 4;
//
//		Vector3 a = GetVertex(ia);
//		Vector3 b = GetVertex(ib);
//		Vector3 c = GetVertex(ic);
//
//		Plane p = Plane::PlaneFromTri(a, b, c);
//
//		Vector3 planePoint = p.ProjectPointOntoPlane(Vector3());
//		if (planePoint.Length() < bestTri) {
//			bestIndex = i;
//			bestPoint = planePoint;
//			bestTri = planePoint.Length();
//		}
//	}
//
//	int ia = bestIndex;
//	int ib = (bestIndex + 1) % 4; //getting the 'older' points on purpose
//	int ic = (bestIndex + 2) % 4;
//
//	Vector3 a = GetVertex(ia);
//	Vector3 b = GetVertex(ib);
//	Vector3 c = GetVertex(ic);
//
//	float triArea = abs(Maths::CrossAreaOfTri(a, b, c));
//
//	float alpha = abs(Maths::CrossAreaOfTri(b, c, bestPoint));
//	float beta  = abs(Maths::CrossAreaOfTri(a, c, bestPoint));
//	float gamma = abs(Maths::CrossAreaOfTri(a, b, bestPoint));
//
//
//	float totalArea = 1.0f / triArea;
//
//	alpha = alpha * totalArea;
//	beta  = beta  * totalArea;
//	gamma = gamma * totalArea;
//
//	float sum = alpha + beta + gamma;
//
//	Vector3 closestPoint = (a * alpha) + (b * beta) + (c * gamma);
//
//	Vector3 closeA = GetSupportPoint(ia).realA *alpha;
//	closeA += GetSupportPoint(ib).onA * beta;
//	closeA += GetSupportPoint(ic).onA * gamma;
//
//	Vector3 closeB = GetSupportPoint(ia).onB *alpha;
//	closeB += GetSupportPoint(ib).onB * beta;
//	closeB += GetSupportPoint(ic).onB * gamma;
//
//	Vector3 realA = GetSupportPoint(ia).realA *alpha;
//	realA += GetSupportPoint(ib).realA * beta;
//	realA += GetSupportPoint(ic).realA * gamma;
//
//	Vector3 realB = GetSupportPoint(ia).realB *alpha;
//	realB += GetSupportPoint(ib).realB * beta;
//	realB += GetSupportPoint(ic).realB * gamma;
//
//	onA = realA;
//	onB = realB;
//}

Vector3 Simplex::BarycentricTriangleWeighting(const Vector3& a, const Vector3&b, const Vector3&c) {
	Plane p = Plane::PlaneFromTri(a, b, c);

	Vector3 planePoint = p.ProjectPointOntoPlane(Vector3());

	float triArea	= abs(Maths::AreaofTri3D(a, b, c));

	float alpha		= abs(Maths::AreaofTri3D(b, c, planePoint));
	float beta		= abs(Maths::AreaofTri3D(a, c, planePoint));
	float gamma		= abs(Maths::AreaofTri3D(a, b, planePoint));

	float totalArea = 1.0f / triArea;

	alpha	= alpha * totalArea;
	beta	= beta  * totalArea;
	gamma	= gamma * totalArea;

	return Vector3(alpha, beta, gamma);
}

//int Simplex::OriginInsideTetrahedron() {
//	if (size != 4) {
//		return 0;
//	}
//
//	int planeState = 0;
//
//	for (int i = 0; i < 4; ++i) {
//		int ia = i;
//		int ib = (i + 1) % 4;
//		int ic = (i + 2) % 4;
//		int id = (i + 3) % 4;
//
//		Vector3 a = GetVertex(ia);
//		Vector3 b = GetVertex(ib);
//		Vector3 c = GetVertex(ic);
//		Vector3 d = GetVertex(id);
//
//		Plane p = Plane::PlaneFromTri(a,b,c);
//
//		float dDistance = p.DistanceFromPlane(d);
//
//		if (dDistance > 0 && p.GetDistance() > 0 ||
//			dDistance < 0 && p.GetDistance() < 0) {
//			planeState |= 1 << i;
//		}
//		else {
//			bool a = true;
//		}
//	}
//	return planeState;
//}

Vector3 Simplex::ClosestPointToTetrahedron(Vector3* realA, Vector3* realB) {
	if (size != 4) {
		return Vector3();
	}
	float bestDist = FLT_MAX;
	Vector3 currentBestPoint;
	Vector3 currentBestBarycentric;

	for (int i = 0; i < 4; ++i) {
		int ia = i;
		int ib = (i + 1) % 4; //getting the 'older' points on purpose
		int ic = (i + 2) % 4;

		Vector3 a = GetVertex(ia);
		Vector3 b = GetVertex(ib);
		Vector3 c = GetVertex(ic);

		Vector3 barycentrics = closestPointToTri(a, b, c);

		Vector3 closestPoint = a * barycentrics.x + b * barycentrics.y + c * barycentrics.z;

		float l = closestPoint.Length();

		if (l < bestDist) {
			currentBestBarycentric	= barycentrics;
			currentBestPoint		= closestPoint;
			bestDist				= l;	
			
			if (realA && realB) {
				*realA = GetSupportPoint(ia).realA * barycentrics.x + GetSupportPoint(ib).realA * barycentrics.y + GetSupportPoint(ic).realA * barycentrics.z;
				*realB = GetSupportPoint(ia).realB * barycentrics.x + GetSupportPoint(ib).realB * barycentrics.y + GetSupportPoint(ic).realB * barycentrics.z;
			}
		}
	}

	return currentBestPoint;
}

//https://slideplayer.com/slide/689954/
Vector3 Simplex::closestPointToTri(const Vector3& a, const Vector3&b, const Vector3& c) {
	//need to take into consideration the voronoi regions of the vertices and edges!

	Vector3 ab = b - a;
	Vector3 ac = c - a;

	float aAB = Vector3::Dot(ab, -a);
	float aAC = Vector3::Dot(ac, -a);

	if (aAB < 0.0f && aAC < 0.0f) { //origin is in voronoi region of vertex A
		return Vector3(1, 0, 0);
	}

	float bAB = Vector3::Dot(ab, -b);
	float bAC = Vector3::Dot(ac, -b);

	if (bAB >= 0.0f && bAC <= bAB) { //origin is in voronoi region of vertex B
		return Vector3(0, 1, 0);
	}

	float cAB = Vector3::Dot(ab, -c);
	float cAC = Vector3::Dot(ac, -c);

	if (cAC > 0.0f && cAB <= cAC) { //origin is in voronoi region of vertex C
		return Vector3(0, 0, 1);
	}

	//form 3 'edge planes' (it's a magic toblerone
	
	Plane p				= Plane::PlaneFromTri(a, b, c);
	Vector3 planePoint  = p.ProjectPointOntoPlane(Vector3());

	Plane pBA = Plane::PlaneFromTri(a, b, b + p.GetNormal());
	Plane pBC = Plane::PlaneFromTri(b, c, c + p.GetNormal());
	Plane pCA = Plane::PlaneFromTri(c, a, a + p.GetNormal());

	if (pBA.GetDistance() < 0 && pBC.GetDistance() < 0 && pCA.GetDistance() < 0) {
		//origin is inside this triangle! do the triangle projection

		Vector3 barycentrics = BarycentricTriangleWeighting(a, b, c);

		return barycentrics;
	}

	if (pBA.GetDistance() > 0) { //outside of edge AB
		Vector3 edgePoint = pBA.ProjectPointOntoPlane(planePoint); //point along AB

		float bLength = (edgePoint - b).Length();
		float aLength = (b - a).Length();

		float ratio = bLength / aLength;
		
		Vector3 barycentrics(1.0f - ratio, ratio, 0);
		return barycentrics;
	}

	if (pBC.GetDistance() > 0) { //outside of edge BC
		Vector3 edgePoint = pBC.ProjectPointOntoPlane(planePoint); //point along BC

		float bLength = (edgePoint - c).Length();
		float aLength = (c - b).Length();

		float ratio = bLength / aLength;

		Vector3 barycentrics(0.0f, 1.0f - ratio, ratio);
		return barycentrics;
	}

	if (pCA.GetDistance() > 0) { //outside of edge AC
		Vector3 edgePoint = pCA.ProjectPointOntoPlane(planePoint); //point along AC

		float bLength = (edgePoint - a).Length();
		float aLength = (a - c).Length();

		float ratio = bLength / aLength;

		Vector3 barycentrics(ratio, 0, 1.0f - ratio);
		return barycentrics;
	}

	return Vector3();
}

Vector3 Simplex::ClosestPoint(Vector3* realA, Vector3* realB) {
	if (size == 1) {
		return GetVertex(0);
	}

	if (size == 2) {
		//find point on line closest to origin

		Vector3 va = GetVertex(0);

		Vector3 ba = GetVertex(1) - va;
		Vector3 ban = ba.Normalised();
		
		Vector3 ca = -va;

		float p = Vector3::Dot(ban, ca.Normalised());

		return va + (ba * p);
	}

	if (size == 3) {
		Vector3 a = GetVertex(0);
		Vector3 b = GetVertex(1);
		Vector3 c = GetVertex(2);

		Vector3 barycentric = closestPointToTri(a, b, c);

		if (realA && realB) {
			*realA = GetSupportPoint(0).realA * barycentric.x + GetSupportPoint(1).realA * barycentric.y + GetSupportPoint(2).realA * barycentric.z;
			*realB = GetSupportPoint(0).realB * barycentric.x + GetSupportPoint(1).realB * barycentric.y + GetSupportPoint(2).realB * barycentric.z;
		}

		return a * barycentric.x + b * barycentric.y + c * barycentric.z;
	}

	if (size == 4) {
		return ClosestPointToTetrahedron(realA, realB);
	}
	return Vector3();
}