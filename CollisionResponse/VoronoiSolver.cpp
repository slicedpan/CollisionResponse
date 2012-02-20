#include "VoronoiSolver.h"
#include "Triangle.h"
#include "ConvexPolyhedron.h"

VoronoiSolver::VoronoiSolver(void)
{
}

VoronoiSolver::~VoronoiSolver(void)
{
}

void VoronoiSolver::Check(ConvexPolyhedron* poly1, ConvexPolyhedron* poly2)
{
	Vec3 p1 = poly1->GetCentre();
	Vec3 p2 = poly2->GetCentre();
	Vec3 d = p1 - p2;

	Triangle* tris1 = poly1->GetTriangles();
	Vec3* points2 = poly2->GetPoints();		

	for (int i = 0; i < poly2->GetNumberOfPoints(); ++i)
	{
		Vec3& point = points2[i];

		if (dot(point - p2, d) < 0)
			continue;

		for (int j = 0; j < poly1->GetNumberOfTriangles(); ++j)
		{
			Triangle& tri = tris1[j];

			if (dot(tri.normal, d) > 0)
				continue;

			int index = VertexVoronoi(point, tri);
			if (index >= 0)
			{
				float curDist = len(point - tri[index]);
				if (curDist < thresholdDistance)
				{
					lastContact.Point = tri[index];
					lastContact.Normal = point - lastContact.Point;
					lastContact.Depth = 0.0f;
					contacts.push_back(lastContact);
					colliding = true;
				}
				continue;	//point is in vertex voronoi region of this triangle
			} 

			index = EdgeVoronoi(point, tri);
			if (index >= 0)
			{
				Vec3 edgePoint;
				Vec3 u = tri[index + 1] - tri[index];
				float l = len(u);
				u.Normalise();
				edgePoint = tri[index] + u * dot(point - tri[index], u);
				float curDist = len(edgePoint - point);
				if (curDist < thresholdDistance)
				{					
					lastContact.Point = edgePoint;
					lastContact.Normal = point - edgePoint;
					lastContact.Depth = 0.0f;
					contacts.push_back(lastContact);
					colliding = true;
				}
				continue;	//point is in edge voronoi region
			}
			//otherwise we are in face voronoi region
			Vec3 facePoint = point - tri.normal * dot(point - tri[0], tri.normal);
			float depth = dot(point - facePoint, tri.normal);
			if (depth < 0)
			{					
				lastContact.Point = facePoint;
				lastContact.Normal = tri.normal;
				lastContact.Depth = -depth;
				contacts.push_back(lastContact);
				colliding = true;
				break;
			}
		}
	}	
}

bool VoronoiSolver::Collide(ConvexPolyhedron* poly1, ConvexPolyhedron* poly2)
{
	minDist = FLT_MAX;
	colliding = false;
	int maxContacts = (poly1->GetNumberOfPoints() / 2) + (poly2->GetNumberOfPoints() / 2) + 2;
	contacts.clear();
	contacts.reserve(maxContacts);
	lastContact.Reversed = 1;
	Check(poly1, poly2);
	lastContact.Reversed = -1;
	Check(poly2, poly1);
	return colliding;
}

int VoronoiSolver::VertexVoronoi(Vec3& point, Triangle& tri)
{
	if (dot((point - tri[0]), (tri[1] - tri[0])) <= 0 && dot(point - tri[0], tri[2] - tri[0]) <= 0)
		return 0;
	if (dot(point - tri[1], tri[2] - tri[1]) <= 0 && dot(point - tri[1], tri[0] - tri[1]) <= 0)
		return 1;
	if (dot(point - tri[2], tri[0] - tri[2]) <= 0 && dot(point - tri[2], tri[1] - tri[2]) <= 0)
		return 2;
	return -1;
}

int VoronoiSolver::EdgeVoronoi(Vec3& point, Triangle& tri)
{
	for (int i = 0; i < 3; ++i)
	{
		if (dot(point - tri[i], tri[i + 1] - tri[i]) < 0 || dot(point - tri[i + 1], tri[i] - tri[i + 1]) < 0)
			continue;
		if (dot(cross(cross(tri[i + 2] - tri[i + 1], tri[i] - tri[i + 1]), tri[i] - tri[i + 1]), point - tri[i + 1]) >= 0)
			return i;
	}
	return -1;
}
