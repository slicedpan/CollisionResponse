#include "DefaultDebugDrawer.h"
#include "AABB.h"
#include "RigidBody.h"
#include "ConvexPolyhedron.h"
#include "Triangle.h"
#include <svl\SVL.h>
#include <glut.h>
#include <svl\SVLgl.h>

void DrawArrow(Vec3& point, Vec3& direction, Vec3& colour)
{	
	Vec3 v1 = Vec3(0.0, 0.0, 1.0);
	if (fabs(dot(direction, v1)) > 0.999f)
		v1 = Vec3(1.0, 0.0, 0.0);
	v1 += direction;
	v1 = cross(v1, direction);
	Vec3 end = point + direction;
	glColor(colour);		
	glPointSize(3.0f);
	glBegin(GL_POINTS);
	glVertex(point);
	glEnd();
	glBegin(GL_LINES);
	glVertex(point);
	glVertex(end);
	glVertex(end);
	glVertex(point + (direction * 0.6) + (v1 * 0.3));
	glVertex(end);
	glVertex(point + (direction * 0.6) - (v1 * 0.3));		
	glEnd();
}

DefaultDebugDrawer::DefaultDebugDrawer(void)
{
}


DefaultDebugDrawer::~DefaultDebugDrawer(void)
{
}

void DefaultDebugDrawer::DrawAABB(AABB& aabb, Vec4& colour)
{
	Vec3* points = aabb.GeneratePoints();
 	glColor(colour);

	glBegin(GL_LINE_STRIP);
	glVertex(points[0]);
	glVertex(points[1]);
	glVertex(points[2]);
	glVertex(points[3]);
	glVertex(points[0]);
	glVertex(points[4]);
	glVertex(points[5]);
	glVertex(points[6]);
	glVertex(points[7]);
	glVertex(points[4]);
	glEnd();

	glBegin(GL_LINES);
	glVertex(points[1]);
	glVertex(points[5]);
	glVertex(points[2]);
	glVertex(points[6]);
	glVertex(points[3]);
	glVertex(points[7]);
	glEnd();

	free(points);
}

void DefaultDebugDrawer::DrawRigidBodyMotion(RigidBody& body)
{
	lastColour = Vec3(1, 0, 0);
	Vec3 vel = body.GetAcceleration(); //approx how far it travels in one sec	
	Contact accVec;
	accVec.Normal = vel;
	accVec.Point = body.GetPosition();
	DrawContact(accVec);
}

void DefaultDebugDrawer::DrawContacts(std::vector<Contact>& contacts)
{
	lastColour = Vec3(0, 0.5, 1);
	for (int i = 0; i < contacts.size(); ++i)
	{
		DrawContact(contacts[i]);
	}
}

void DefaultDebugDrawer::DrawContact(Contact& contact)
{
	DrawArrow(contact.Point, contact.Normal, lastColour);
}

void DefaultDebugDrawer::DrawPoly(ConvexPolyhedron* poly)
{
	Triangle* tris = poly->GetTriangles();
	Vec3* points = poly->GetPoints();
	glPointSize(3.0f);
	
	for (int i = 0; i < poly->GetNumberOfTriangles(); ++i)
	{
		lastColour = tris[i].debugColour;
		glColor(tris[i].debugColour);
		Vec3& p1 = points[tris[i].point[0]];
		Vec3& p2 = points[tris[i].point[1]];
		Vec3& p3 = points[tris[i].point[2]];
		glBegin(GL_POINTS);
		glVertex(p1);
		glVertex(p2);
		glVertex(p3);
		glEnd();
		glBegin(GL_LINE_STRIP);
		glVertex(p1);
		glVertex(p2);
		glVertex(p3);
		glVertex(p1);
		glEnd();
		Contact contact(tris[i].normal, (p1 + p2 + p3) / 3);
		DrawContact(contact);
	}	
	glColor3f(0, 0.5, 1);
	glBegin(GL_POINTS);
	glVertex(poly->GetCentre());
	glEnd();	
}

void DefaultDebugDrawer::DrawTriMesh(Triangle* tri, int triCount, Vec4& colour)
{
	glColor(colour);
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < triCount; ++i)
	{
		glVertex(tri[i][0]);
		glVertex(tri[i][1]);
		glVertex(tri[i][2]);
	}
	glEnd();
}

