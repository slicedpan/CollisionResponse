#include "RigidBody.h"
#include "Contact.h"

RigidBody::RigidBody(void) : 
	position(0, 0, 0), 
	lastPosition(0, 0, 0), 
	acceleration(0, 0, 0), 
	invMass(1),
	orientation(1.0, 0.0, 0.0, 0.0),
	angularVelocity(1.0, 0.0, 0.0, 0.0),
	debugColour(1.0, 1.0, 1.0, 1.0),
	isKinematic(false),
	restitutionCoefficient(0.3f)
{
	transform.MakeDiag();
	invInertiaTensor.MakeDiag();
	rotationMatrix.MakeDiag();
	baseBB.SetMax(Vec3(1, 1, 1));
	baseBB.SetMin(Vec3(-1, -1, -1));
	CalculateTransform();
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
	ApplyAngularImpulse(axisAngle, len(axisAngle));
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
	float impulse;
	Vec3 offset1, offset2;
	float relativeVelocity;
	Mat3 Iinv1 = body1->GetOrientationMatrix() * body1->GetInverseInertiaTensor() * trans(body1->GetOrientationMatrix());
	Mat3 Iinv2 = body2->GetOrientationMatrix() * body2->GetInverseInertiaTensor() * trans(body2->GetOrientationMatrix());
	Vec3 pointVel1, pointVel2;
	Vec3 rotationAxis1 = qAxisAngle(body1->GetOrientation());
	Vec3 rotationAxis2 = qAxisAngle(body2->GetOrientation());
	float t3, t4;
	float restitution = (body1->GetRestitution() + body2->GetRestitution()) / 2.0f;
	Vec3 impulseVec;
	Vec3 angularAcc;
	
	for (int i = 0; i < contacts.size(); ++i)
	{		
		offset1 = contacts[i].Point - body1->GetPosition();
		offset2 = contacts[i].Point - body2->GetPosition();		
		pointVel1 = body1->GetVelocity() + cross(rotationAxis1, offset1);
		pointVel2 = body2->GetVelocity() + cross(rotationAxis2, offset2);
		relativeVelocity = dot(contacts[i].Normal, (pointVel1 - pointVel2));
		t3 = dot(contacts[i].Normal, cross((Iinv1 * cross(offset1, contacts[i].Normal)), offset1));
		t4 = dot(contacts[i].Normal, cross((Iinv2 * cross(offset2, contacts[i].Normal)), offset2));
		impulse = (-(1 + restitution) * relativeVelocity) / (body1->GetInverseMass() + body2->GetInverseMass() + t3 + t4);
		impulseVec = impulse * contacts[i].Normal * contacts[i].Reversed;
		body1->ApplyForce(impulseVec);
		body2->ApplyForce(-impulseVec);

		angularAcc = Iinv1 * cross(offset1, impulseVec);
		body1->ApplyAngularImpulse(angularAcc);
		angularAcc = Iinv2 * cross(offset2, -impulseVec);
		body2->ApplyAngularImpulse(angularAcc);
	}
}
