#include "RigidBody.h"
#include "Contact.h"

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

	float impulse;
	Vec3 offset1, offset2;
	float relativeVelocity;
	Mat3 Iinv1 = body1->GetOrientationMatrix() * body1->GetInverseInertiaTensor() * trans(body1->GetOrientationMatrix());
	Mat3 Iinv2 = body2->GetOrientationMatrix() * body2->GetInverseInertiaTensor() * trans(body2->GetOrientationMatrix());
	Vec3 pointVel1, pointVel2;
	Vec3 rotationAxis1 = qAxisAngle(body1->GetAngularVelocity());
	Vec3 rotationAxis2 = qAxisAngle(body2->GetAngularVelocity());
	float t3, t4;
	float restitution = 1.0f;
	Vec3 impulseVec;
	Vec3 angularAcc;
	Vec3 body1Impulse(0.0, 0.0, 0.0);
	Vec3 body2Impulse(0.0, 0.0, 0.0);

	Vec3 spotDistance = contacts[0].Depth * contacts[0].Normal;
	spotDistance *= 1.001;

	

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
	
	for (int i = 0; i < contacts.size(); ++i)
	{		
		++numContacts;
		offset1 = contacts[i].Point - body1->GetPosition();
		offset2 = contacts[i].Point - body2->GetPosition();		
		pointVel1 = body1->GetVelocity() + cross(rotationAxis1, offset1);
		pointVel2 = body2->GetVelocity() + cross(rotationAxis2, offset2);
		contacts[i].Normal *= contacts[i].Reversed;
		relativeVelocity = dot(contacts[i].Normal, (pointVel1 - pointVel2));
		relativeVel = relativeVelocity;
		//relativeVelocity = 0.1f;
		t3 = dot(contacts[i].Normal, cross((Iinv1 * cross(offset1, contacts[i].Normal)), offset1));
		t4 = dot(contacts[i].Normal, cross((Iinv2 * cross(offset2, contacts[i].Normal)), offset2));

		impulse = (-(1 + restitution) * relativeVelocity) / (body1->GetInverseMass() + body2->GetInverseMass() + t3 + t4);
		impulseVec = impulse * contacts[i].Normal;

		/*body1Impulse += impulseVec;
		body2Impulse -= impulseVec;*/

		body1->ApplyImpulse(impulseVec);
		body2->ApplyImpulse(-impulseVec);

		angularAcc = Iinv1 * cross(offset1, -impulseVec);
		body1->ApplyAngularImpulse(angularAcc);
		angularAcc = Iinv2 * cross(offset2, impulseVec);
 		body2->ApplyAngularImpulse(angularAcc);
	}
	/*body1->ApplyImpulse(body1Impulse / contacts.size());
	body2->ApplyImpulse(body2Impulse / contacts.size());*/
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