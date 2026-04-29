#include <raylib.h>
#include <Jolt/Jolt.h>

inline Vector3 operator*(const float& v, const Vector3& vr)
{
	return {vr.x * v, vr.y * v, vr.z * v};
}

inline Vector3 toRayVec(JPH::Vec3 v)
{
	return {v.GetX(), v.GetY(), v.GetZ()};
}