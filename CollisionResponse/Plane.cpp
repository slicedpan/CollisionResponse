#include "Plane.h"
#include <glut.h>
#include <svl\SVLgl.h>
#include "Contact.h"

Plane::Plane(Vec3 normal, Vec3 position) : ConvexPolyhedron(4, 2)
{
	this->normal = norm(normal);
	this->position = position;
	v1 = normal + Vec3(1.0, 0.0, 0.0);
	v2 = norm(cross(v1, normal));
	v1 = norm(cross(v2, normal));
	SetPosition(position);
	localPoints[0] = (v1 + v2) * 100.0f;
	localPoints[1] = (-v1 + v2) * 100.0f;
	localPoints[2] = (-v1 - v2) * 100.0f;
	localPoints[3] = (v1 - v2) * 100.0f;
	int indices[] = {0, 1, 2, 0, 2, 3};
	InitialiseTris(indices);
	Vec3 planeBoxPoints[12];
	memcpy(planeBoxPoints, localPoints, sizeof(Vec3) * 4);
	memcpy(planeBoxPoints + 4, localPoints, sizeof(Vec3) * 4);
	memcpy(planeBoxPoints + 8, localPoints, sizeof(Vec3) * 4);
	for (int i = 4; i < 8; ++i)
	{
		planeBoxPoints[i] += (this->normal * 5.0f);
	}
	for (int i = 8; i < 12; ++i)
	{
		planeBoxPoints[i] -= (this->normal * 5.0f);
	}
	baseBB.InitFromPoints(planeBoxPoints, 12);
	SetRigidBody(this);
}

Plane::~Plane(void)
{
}

Vec3 Plane::GetRestoringForce(Vec3 point)
{
	return -normal * GetDistanceTo(point);
}

Contact * Plane::GetContact(Vec3& point)
{
	Contact * contact = new Contact();
	contact->Point = point - dot(normal, point) * normal;
	contact->Normal = normal;
	return contact;
}

void Plane::Draw()
{
	glDisable(GL_LIGHTING);
	glBegin(GL_LINES);
		glColor3f(1.0, 0.0, 0.0);
		glVertex(position);
		glVertex(position + Vec3(1, 0, 0));
		glColor3f(0.0, 1.0, 0.0);
		glVertex(position);
		glVertex(position + Vec3(0, 0, 1));
		glColor3f(0.0, 0.0, 1.0);
		glVertex(position);
		glVertex(position + normal);
	glEnd();

	/*
	glEnable(GL_LIGHTING);
	glBegin(GL_QUADS);
		glColor3f(0.3, 0.3, 0.3);
		glVertex(position + (v1 - v2) * 100.0f);
		glVertex(position + (v1 + v2) * 100.0f);
		glVertex(position + (-v1 + v2) * 100.0f);
		glVertex(position + (-v1 - v2) * 100.0f);
	glEnd();
	*/
}
