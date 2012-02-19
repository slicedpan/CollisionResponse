#pragma once

#include "INarrowPhaseSolver.h"
#include "Contact.h"
#include <vector>

class ConvexPolyhedron;
struct Triangle;

class VoronoiSolver : public INarrowPhaseSolver
{
public:
	VoronoiSolver(void);
	~VoronoiSolver(void);
	bool Collide(ConvexPolyhedron* poly1, ConvexPolyhedron* poly2);		
	void SetThresholdDistance(float value) { thresholdDistance = value; }
	std::vector<Contact>& GetContacts() { return contacts; }
private:
	std::vector<Contact> contacts;
	Contact lastContact;
	void Check(ConvexPolyhedron* poly1, ConvexPolyhedron* poly2);
	int VertexVoronoi(Vec3& point, Triangle& tri);
	int EdgeVoronoi(Vec3& point, Triangle& tri);
	float minDist;
	bool colliding;
	float thresholdDistance;
};

