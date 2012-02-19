#include "NarrowPhase.h"
#include "PhysicsSystem.h"
#include "VoronoiSolver.h"
#include "ConvexPolyhedron.h"
#include "RigidBody.h"
#include <algorithm>

NarrowPhase::NarrowPhase(void)
{
	solver = new VoronoiSolver();
}

NarrowPhase::~NarrowPhase(void)
{
}

void NarrowPhase::CollidePairs(std::vector<NarrowPhasePair>& pairs)
{
	RigidBody* body1, *body2;
	contacts.clear();
	if (pairs.size() == 0)
		return;
	for (int i = 0; i < pairs.size(); ++i)
	{		
		if (solver->Collide(pairs[i].p1, pairs[i].p2))
		{			
			std::vector<Contact>& pairContacts = solver->GetContacts();			
			std::sort(pairContacts.begin(), pairContacts.end(), DepthSort);
			contacts.insert(contacts.end(), pairContacts.begin(), pairContacts.end());
			body1 = pairs[i].p1->GetRigidBody();
			body2 = pairs[i].p2->GetRigidBody();
			pairs[i].p1->OnNarrowPhase(pairs[i].p2, pairContacts);
			pairs[i].p2->OnNarrowPhase(pairs[i].p1, pairContacts);

			if (body1 && body2)
			{
				RigidBody::ApplyContactImpulses(pairContacts, body1, body2);
			}
		}

	}
}
