#pragma once

namespace NCL {
	namespace Maths {
		class Simplex
		{
		public:

			struct SupportPoint {
				Vector3 pos;
				Vector3 onA;
				Vector3 onB;
				Vector3 realA;
				Vector3 realB;
			};

			Simplex();
			~Simplex();

			void SetToPoint(SupportPoint a);

			void SetToTri(SupportPoint a, SupportPoint b, SupportPoint c);

			void SetToLine(SupportPoint a, SupportPoint b);

			void Add(SupportPoint a);

			int GetSize() const {
				return size;
			}

			Vector3 GetVertex(int i) const {
				return verts[IndexToSlot(i)].pos;
			}

			Vector3 GetNormal(int i) const {
				Vector3 a = verts[IndexToSlot(i)].pos;
				Vector3 b = verts[IndexToSlot(i-1)].pos;
				Vector3 c = verts[IndexToSlot(i-2)].pos;

				Vector3 ba = b - a;
				Vector3 ca = c - a;

				Vector3 n = Vector::Normalise(Vector::Cross(ba, ca));

				return n;
			}


			SupportPoint GetSupportPoint(int i) const {
				return verts[IndexToSlot(i)];
			}

			void RemoveOldestSupportPoint();

			float GetClosestDistance();

			//void DetermineBestCollisions(Vector3& onA, Vector3& onB);

			bool ExtractCollisionInfo(Vector3& normal, float& penetration);

			int OriginInsideTetrahedron();
			Vector3 ClosestPointToTetrahedron(Vector3* realA, Vector3* realB);

			Vector3 closestPointToTri(const Vector3& a, const Vector3&b, const Vector3& c);

			Vector3 BarycentricTriangleWeighting(const Vector3& a, const Vector3&b, const Vector3&c);

			Vector3 ClosestPoint(Vector3*a = nullptr, Vector3*b = nullptr);

		protected:


			int IndexToSlot(int i) const {
				int slot = (index - 1) - i;
				if (slot < 0) {
					slot += size;
				}
				return slot;
			}

			SupportPoint verts[4];
			int index;
			int size;
		};
	}
}

