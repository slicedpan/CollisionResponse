#pragma once

#include "Contact.h"
#include <vector>

class ConvexPolyhedron;

class INarrowPhaseSolver
{
public:
	virtual bool Collide(ConvexPolyhedron* p1, ConvexPolyhedron* p2) = 0;
	virtual std::vector<Contact>& GetContacts() = 0;
};

