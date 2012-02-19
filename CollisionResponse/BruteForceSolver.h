#pragma once

#include "INarrowPhaseSolver.h"

struct Triangle;

class BruteForceSolver : public INarrowPhaseSolver
{
public:
	BruteForceSolver(void);
	~BruteForceSolver(void);
	bool Collide(ConvexPolyhedron* p1, ConvexPolyhedron* p2);
	std::vector<Contact>& GetContacts() { return contacts; }
private:
	bool Intersects(Vec3& point, Triangle& tri);
	std::vector<Contact> contacts;
};

