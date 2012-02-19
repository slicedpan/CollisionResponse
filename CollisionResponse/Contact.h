#pragma once

#include <svl\SVL.h>

struct Contact
{
	Contact(Vec3& Normal, Vec3& Point)
	{
		this->Normal = Normal;
		this->Point = Point;
	}
	Contact()
	{
		Normal.MakeZero();
		Point.MakeZero();
	}
	Vec3 Normal;
	Vec3 Point;
	bool Reversed;
	float Depth;
};

inline bool DepthSort(Contact& c1, Contact& c2)
{
	return c1.Depth > c2.Depth;
}

