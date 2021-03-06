#include "PhysicsSystem.h"
#include "ICollidable.h"
#include "Contact.h"
#include "RigidBody.h"
#include "IDebugDrawer.h"
#include "ConvexPolyhedron.h"

PhysicsSystem * PhysicsSystem::currentInstance;

PhysicsSystem::PhysicsSystem(void) : debugDrawer(0)
{
}

PhysicsSystem::~PhysicsSystem(void)
{
}

void PhysicsSystem::DrawDebug()
{
	if (!debugDrawer)
		return;
	for (int i = 0; i < rigidBodies.size(); ++i)
	{
		debugDrawer->DrawAABB(rigidBodies[i]->GetAABB(), rigidBodies[i]->GetDebugColour());		
		debugDrawer->DrawRigidBodyMotion(*rigidBodies[i]);
		ConvexPolyhedron* poly = rigidBodies[i]->GetPoly();
		if (poly)
		{
			debugDrawer->DrawPoly(poly);
			debugDrawer->DrawTriMesh(poly->GetTriangles(), poly->GetNumberOfTriangles(), poly->GetDebugColour());
		}
	}
	debugDrawer->DrawContacts(narrowPhase.GetContacts());
}

void PhysicsSystem::AddCollidable(ICollidable* obj)
{
	collidables.push_back(obj);
}

ICollidable* PhysicsSystem::CollideWith(Vec3 point)
{
	for (int i = 0; i < collidables.size(); ++i)
	{
		if (collidables[i]->PointIntersects(point))
			return collidables[i];
	}
	return 0;
}

void PhysicsSystem::Integrate(float timeStep)
{	
	float timeSquared = timeStep * timeStep;
	step = timeStep;
	Vec4 avel;

	for (int i = 0; i < rigidBodies.size(); ++i)
	{
		rigidBodies[i]->SetDebugColour(Vec4(1, 1, 1, 1));
		if (!rigidBodies[i]->IsKinematic())
		{
			rigidBodies[i]->ClearAcceleration();
			rigidBodies[i]->ApplyForce(Vec3(0, -1, 0)); //Gravity
			rigidBodies[i]->UpdatePosition((2 * rigidBodies[i]->GetPosition()) - rigidBodies[i]->GetLastPosition() + rigidBodies[i]->GetAcceleration() * timeSquared);
			avel = rigidBodies[i]->GetAngularVelocity();			
			rigidBodies[i]->SetOrientation(qMultiply(rigidBodies[i]->GetOrientation(), avel * timeStep));			
			rigidBodies[i]->CalculateTransform();
			rigidBodies[i]->CalculateBB();
			rigidBodies[i]->OnUpdateTransform();			
		}
	}	
	broadPhase.GenerateCollisions();
	std::vector<BroadPhasePair>& pairs = broadPhase.GetPairs();
	std::vector<NarrowPhasePair> nPairs;
	for (int i = 0; i < pairs.size(); ++i)
	{
		if (pairs[i].p1->OnBroadPhaseCollide(pairs[i].p2) && pairs[i].p2->OnBroadPhaseCollide(pairs[i].p1))
		{
			ConvexPolyhedron* poly1 = pairs[i].p1->GetPoly();
			ConvexPolyhedron* poly2 = pairs[i].p2->GetPoly();
			if (poly1 && poly2)
				nPairs.push_back(NarrowPhasePair(poly1, poly2));
		}
	}
	narrowPhase.CollidePairs(nPairs);	
}

void PhysicsSystem::AddRigidBody(RigidBody* bodyToAdd)
{
	rigidBodies.push_back(bodyToAdd);
	broadPhase.AddBody(bodyToAdd);
	bodyToAdd->CalculateTransform();
	bodyToAdd->CalculateBB();
	bodyToAdd->OnUpdateTransform();
}

void PhysicsSystem::SetDebugDrawer(IDebugDrawer* debugDrawer)
{
	this->debugDrawer = debugDrawer;
}

