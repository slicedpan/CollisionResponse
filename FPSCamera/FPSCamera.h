#pragma once

#include "svl/svl.h"

class FPSCamera
{
public:
	FPSCamera(void);
	~FPSCamera(void);
	Mat4 GetTransform();
	Mat4 GetViewTransform();
	Mat4 GetProjectionMatrix();
	void SetAspectRatio(float aspectRatio) { this->aspectRatio = aspectRatio; }
	void SetFOV(float fieldOfView) { fov = fieldOfView; }
	void SetZNear(float zNear) { this->zNear = zNear; }
	void SetZFar(float zFar) { this->zFar = zFar; }
	float Pitch, Yaw;
	Vec3 Position;
	Vec3 GetForwardVector();
	Vec3 GetRightVector();
private:
	float aspectRatio;
	float fov;
	float zNear;
	float zFar;
};

