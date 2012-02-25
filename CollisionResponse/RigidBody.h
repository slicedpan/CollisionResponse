#pragma once

#include <svl\SVL.h>
#include "AABB.h"
#include <vector>

class ConvexPolyhedron;
struct Contact;

extern int impulseCount;

class RigidBody
{
public:
	RigidBody(void);
	~RigidBody(void);
	void ApplyImpulse(Vec3& impulse);
	void ApplyForce(Vec3& force);
	void ApplyForceAtPoint(Vec3& force, Vec3& point);
	void SetInverseMass(float invMass) { this->invMass = invMass; }
	float GetInverseMass() { return invMass; }
	void SetRestitution(float value) { restitutionCoefficient = value; }
	float GetRestitution() { return restitutionCoefficient; }
	void SetPosition(Vec3& position);
	void UpdatePosition(Vec3& position);
	Vec3& GetVelocity();
	Vec3& GetPosition();
	Vec3& GetAcceleration();
	Vec3& GetLastPosition();
	Vec4& GetAngularVelocity();
	Vec4& GetOrientation();
	Mat3& GetOrientationMatrix() { return rotationMatrix; }
	Mat3& GetInverseInertiaTensor() { return invInertiaTensor; }
	void MoveBy(Vec3& offset) { SetPosition(position + offset); }
	void SetOrientation(Vec4& orientation);
	void ApplyAngularImpulse(Vec4& angularVelocity);
	void ApplyAngularImpulse(Vec3& axis, float amount);
	void ApplyAngularImpulse(Vec3& axisAngle);
	void ClearAcceleration();
	void CalculateTransform();
	void CalculateBB();
	void ApplyContact(Contact& contact);
	Vec4& GetDebugColour();
	void SetDebugColour(Vec4& colour);
	Mat4& GetTransform();
	AABB& GetAABB();
	virtual void OnUpdateTransform() {}
	virtual bool OnBroadPhaseCollide(RigidBody* other) { return false; }	//true to pass to narrowphase
	virtual ConvexPolyhedron* GetPoly() { return 0; }
	void ApplyContactImpulse(std::vector<Contact>& contacts, RigidBody* other);
	void SetKinematic(bool kinematic);
	bool IsKinematic() { return isKinematic; }
	static void ApplyContactImpulses(std::vector<Contact>& contacts, RigidBody* body1, RigidBody* body2);
protected:
	AABB baseBB;
	Vec4 debugColour;	
	Mat3 invInertiaTensor;
private:
	Vec3 position;
	Vec3 lastPosition;
	Vec3 acceleration;
	Vec3 velocity;
	Vec4 orientation;
	Vec4 angularVelocity;
	Mat4 transform;	
	Mat3 rotationMatrix;
	AABB currentBB;
	float invMass;
	ConvexPolyhedron* poly;
	void GenerateRotationMatrix();	
	float restitutionCoefficient;
	bool isKinematic;

};

inline void RigidBody::SetPosition(Vec3& position)
{	
	GetVelocity();	
	this->position = position;
	lastPosition = position - velocity;
	CalculateTransform();
}

inline void RigidBody::UpdatePosition(Vec3& position)
{
	lastPosition = this->position;
	this->position = position;
}

inline void RigidBody::ClearAcceleration()
{
	acceleration.MakeZero();
}

inline Vec3& RigidBody::GetVelocity() 
{
	if (isKinematic)
		return Vec3(0.0, 0.0, 0.0);
	velocity = position - lastPosition;	
	return velocity;
}

inline Vec3& RigidBody::GetPosition() {return position;}
inline Vec3& RigidBody::GetAcceleration() {return acceleration;}
inline Vec3& RigidBody::GetLastPosition() {return lastPosition;}

inline Vec4& RigidBody::GetOrientation() {return orientation;}
inline Vec4& RigidBody::GetAngularVelocity() {return angularVelocity;}

inline void RigidBody::SetDebugColour(Vec4& colour) {debugColour = colour;}
inline Vec4& RigidBody::GetDebugColour() {return debugColour;}

inline void RigidBody::ApplyForce(Vec3& force)
{
	acceleration += force * invMass;
}

inline void RigidBody::ApplyImpulse(Vec3& impulse)
{
	lastPosition -= impulse;
	++impulseCount;
}

inline void RigidBody::ApplyAngularImpulse(Vec4& angularImpulse)
{
	angularVelocity = qMultiply(angularVelocity, angularImpulse);
}

inline void RigidBody::SetOrientation(Vec4& orientation)
{
	this->orientation = orientation;
}




