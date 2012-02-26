#include "RigidBody.h"
#include "Contact.h"
#include "PhysicsSystem.h"

extern int numContacts;
extern float relativeVel;
extern bool breakOnImpulse;

RigidBody::RigidBody(void) : 
	position(0, 0, 0), 
	lastPosition(0, 0, 0), 
	acceleration(0, 0, 0), 
	invMass(1),
	orientation(1.0, 0.0, 0.0, 0.0),
	angularVelocity(1.0, 0.0, 0.0, 0.0),
	debugColour(1.0, 1.0, 1.0, 1.0),
	isKinematic(false),
	restitutionCoefficient(0.3f),
	velocity(0.0, 0.0, 0.0)
{
	transform.MakeDiag();
	invInertiaTensor.MakeDiag();
	rotationMatrix.MakeDiag();
	baseBB.SetMax(Vec3(1, 1, 1));
	baseBB.SetMin(Vec3(-1, -1, -1));
}

RigidBody::~RigidBody(void)
{

}

void RigidBody::ApplyContact(Contact& contact)
{
	Vec3 vel = dot(contact.Normal, velocity) * contact.Normal;
	ApplyImpulse(-2 * vel);
}

Mat4& RigidBody::GetTransform()
{
	return transform;
}

void RigidBody::CalculateTransform()
{
	angularVelocity.Normalise();
	orientation.Normalise();
	transform = qGetTransform(orientation) * HTrans4(position);
	GenerateRotationMatrix();
}

void RigidBody::CalculateBB()
{
	currentBB = baseBB.Transform(transform);
}

AABB& RigidBody::GetAABB()
{
	return currentBB;
}

void RigidBody::ApplyAngularImpulse(Vec4& angularImpulse)
{
	angularVelocity = qMultiply(angularVelocity, angularImpulse);
}

void RigidBody::ApplyAngularImpulse(Vec3& axis, float amount)
{
	float s = sin(amount);
	axis.Normalise();
	Vec4 quat = Vec4(cos(amount), s * axis[0], s * axis[1], s * axis[2]);
	ApplyAngularImpulse(quat);
}

void RigidBody::ApplyAngularImpulse(Vec3& axisAngle)
{
	float amount = len(axisAngle);
	if (amount == 0.0f)
		return;
	ApplyAngularImpulse(axisAngle, amount);
}

void RigidBody::ApplyContactImpulse(std::vector<Contact>& contacts, RigidBody* other)
{
	if (isKinematic)
		return;
}

void RigidBody::GenerateRotationMatrix()
{
	memcpy(rotationMatrix.Ref(), transform.Ref(), sizeof(Real) * 3);
	memcpy(rotationMatrix.Ref() + 3, transform.Ref() + 4, sizeof(Real) * 3);
	memcpy(rotationMatrix.Ref() + 6, transform.Ref() + 8, sizeof(Real) * 3);
}

void RigidBody::ApplyContactImpulses(std::vector<Contact>& contacts, RigidBody* body1, RigidBody* body2)
{

	if (breakOnImpulse)
		int i = 0;

 	int contactNum = contacts.size();
	float impulse;
	Vec3 offset1, offset2;
	float relativeVelocity;
	Mat3 Iinv1 = body1->GetOrientationMatrix() * body1->GetInverseInertiaTensor() * trans(body1->GetOrientationMatrix());
	Mat3 Iinv2 = body2->GetOrientationMatrix() * body2->GetInverseInertiaTensor() * trans(body2->GetOrientationMatrix());
	Vec3 pointVel1(0.0, 0.0, 0.0), pointVel2(0.0, 0.0, 0.0);
	Vec3 rotationAxis1 = qAxisAngle(body1->GetAngularVelocity());
	Vec3 rotationAxis2 = qAxisAngle(body2->GetAngularVelocity());
	float t3, t4;
	float restitution = 0.2f;
	float friction = 0.5f;
	Vec3 impulseVec(0.0, 0.0, 0.0);
	Vec3 angularAcc;
	Vec3 body1Impulse(0.0, 0.0, 0.0);
	Vec3 body2Impulse(0.0, 0.0, 0.0);

	Vec3 spotDistance = contacts[0].Depth * contacts[0].Normal;
	spotDistance *= 1.000001f;	

	if (dot(spotDistance, body1->GetPosition() - body2->GetPosition()) > 0.0)
	{
		
	}

	if (body1->IsKinematic())
	{
		body2->MoveBy(-spotDistance);
	}
	else if (body2->IsKinematic()) 
	{
		body1->MoveBy(spotDistance);
	}
	else
	{
		body2->MoveBy(spotDistance * -0.5);
		body1->MoveBy(spotDistance * 0.5);
	}

	Vec3 avgPoint(0.0, 0.0, 0.0);
	Vec3 avgNormal(0.0, 0.0, 0.0);

	Vec3 velocityDifference;
	Vec3 transverseImpulse(0.0, 0.0, 0.0);

	float maxDepth = -FLT_MAX;

	for (int i = 0; i < contactNum; ++i)
	{
		avgPoint += contacts[i].Point;
		contacts[i].Normal *= contacts[i].Reversed;
		avgNormal += contacts[i].Normal;

		offset1 = contacts[i].Point - body1->GetPosition();
		offset2 = contacts[i].Point - body2->GetPosition();
		
		if (contacts[i].Depth > maxDepth)
			maxDepth = contacts[i].Depth;

		Vec3 rotVel1 = cross(rotationAxis1, offset1);
		Vec3 rotVel2 = cross(rotationAxis2, offset2);

		velocityDifference = rotVel1 - rotVel2 + body1->GetVelocity() - body2->GetVelocity();

		Vec3 transverseDir = cross(cross(contacts[i].Normal, velocityDifference), contacts[i].Normal);
		float transverseDirLength = len(transverseDir);
		if (transverseDirLength > 0.0000001f)
			transverseDir /= transverseDirLength;
		float transverseVel = dot(velocityDifference, transverseDir);

		angularAcc = Iinv1 * cross(offset1, -transverseVel * friction * transverseDir);
		body1->ApplyAngularImpulse(angularAcc);
		angularAcc = Iinv2 * cross(offset2, transverseVel * friction * transverseDir);
		body2->ApplyAngularImpulse(angularAcc);

		transverseImpulse += transverseVel * friction * transverseDir;

		pointVel1 += rotVel1;
		pointVel2 += rotVel2;
	}

	avgPoint /= contactNum;
	avgNormal /= contactNum;
	transverseImpulse /= contactNum;

	contacts[0].Normal = avgNormal;
	contacts[0].Point = avgPoint;	
			
	++numContacts;

	float projectedPenetration = dot(body1->GetAcceleration(), avgNormal) - dot(body2->GetAcceleration(), avgNormal);
	projectedPenetration *= pow(PhysicsSystem::GetCurrentInstance()->GetTimeStep(), 2.0f);

	if (projectedPenetration <= maxDepth + 0.0000001f)
	{
		offset1 = avgPoint - body1->GetPosition();
		offset2 = avgPoint - body2->GetPosition();		
		pointVel1 += body1->GetVelocity();
		pointVel2 += body2->GetVelocity();
		velocityDifference = pointVel1 - pointVel2;
		relativeVelocity = dot(avgNormal, velocityDifference);
		relativeVel = relativeVelocity;	
		//relativeVelocity = 0.1f;
		t3 = dot(avgNormal, cross((Iinv1 * cross(offset1, avgNormal)), offset1));
		t4 = dot(avgNormal, cross((Iinv2 * cross(offset2, avgNormal)), offset2));

		impulse = (-(1 + restitution) * relativeVelocity) / (body1->GetInverseMass() + body2->GetInverseMass() + t3 + t4);
		if (impulse > 0)	//separating contact
			impulse = 0;
		impulse += (projectedPenetration * 2.0f);
		if (fabs(impulse) > 0.000001f)		
			impulseVec = impulse * avgNormal;
	}
	
	body1->ApplyImpulse(impulseVec - transverseImpulse);
	body2->ApplyImpulse(-impulseVec + transverseImpulse);	

	angularAcc = Iinv1 * cross(offset1, -impulseVec);
	body1->ApplyAngularImpulse(angularAcc / 2.0);
	angularAcc = Iinv2 * cross(offset2, impulseVec);
 	body2->ApplyAngularImpulse(angularAcc / 2.0);
	
}

void RigidBody::SetKinematic(bool kinematic)
{
	isKinematic = kinematic;
	if (isKinematic)
	{
		invMass = 0;
		invInertiaTensor.MakeZero();
		lastPosition = position;
		velocity.MakeZero();
	}
}

void RigidBody::ApplyForceAtPoint(Vec3& force, Vec3& point)
{
	Mat3 Iinv = rotationMatrix * invInertiaTensor * trans(rotationMatrix);
	Vec3 offset = point - position;
	Vec3 angularAcc = Iinv * cross(offset, force);
	ApplyAngularImpulse(angularAcc);
}
