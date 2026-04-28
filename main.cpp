// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2025 Jorrit Rouwe
// SPDX-License-Identifier: CC0-1.0
// This file is in the public domain. It serves as an example to start building your own application using Jolt Physics. Feel free to copy paste without attribution!

// The Jolt headers don't include Jolt.h. Always include Jolt.h before including any other Jolt header.
// You can use Jolt.h in your precompiled header to speed up compilation.
#include <Jolt/Jolt.h>

// Jolt includes
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>

#include "Jolt/Core/Core.h"
#include "Jolt/Math/MathTypes.h"
#include "Jolt/Math/Quat.h"
#include "Jolt/Math/Vec3.h"
#include "Jolt/Physics/Body/BodyID.h"
#include "Jolt/Physics/Body/BodyInterface.h"
#include "Jolt/Physics/Body/MotionType.h"
#include "Jolt/Physics/Collision/ObjectLayer.h"
#include "Jolt/Physics/EActivation.h"
#include "raylib.h"
#include "rlgl.h"

// STL includes
#include <complex>
#include <cstddef>
#include <iostream>
#include <cstdarg>
#include <optional>
#include <thread>
#include <vector>

// Disable common warnings triggered by Jolt, you can use JPH_SUPPRESS_WARNING_PUSH / JPH_SUPPRESS_WARNING_POP to store and restore the warning state
JPH_SUPPRESS_WARNINGS

using namespace std;

static void TraceImpl(const char *inFMT, ...)
{
	va_list list;
	va_start(list, inFMT);
	char buffer[1024];
	vsnprintf(buffer, sizeof(buffer), inFMT, list);
	va_end(list);

	// Print to the TTY
	cout << buffer << endl;
}

#ifdef JPH_ENABLE_ASSERTS

// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, JPH::uint inLine)
{
	// Print to the TTY
	cout << inFile << ":" << inLine << ": (" << inExpression << ") " << (inMessage != nullptr? inMessage : "") << endl;

	// Breakpoint
	return true;
};

#endif // JPH_ENABLE_ASSERTS

// Layer that objects can be in, determines which other objects it can collide with
// Typically you at least want to have 1 layer for moving bodies and 1 layer for static bodies, but you can have more
// layers if you want. E.g. you could have a layer for high detail collision (which is not used by the physics simulation
// but only if you do collision testing).
namespace Layers
{
	static constexpr JPH::ObjectLayer NON_MOVING = 0;
	static constexpr JPH::ObjectLayer MOVING = 1;
	static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
};

using namespace JPH::literals;

/// Class that determines if two object layers can collide
class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter
{
public:
	virtual bool					ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override
	{
		switch (inObject1)
		{
		case Layers::NON_MOVING:
			return inObject2 == Layers::MOVING; // Non moving only collides with moving
		case Layers::MOVING:
			return true; // Moving collides with everything
		default:
			JPH_ASSERT(false);
			return false;
		}
	}
};

// Each broadphase layer results in a separate bounding volume tree in the broad phase. You at least want to have
// a layer for non-moving and moving objects to avoid having to update a tree full of static objects every frame.
// You can have a 1-on-1 mapping between object layers and broadphase layers (like in this case) but if you have
// many object layers you'll be creating many broad phase trees, which is not efficient. If you want to fine tune
// your broadphase layers define JPH_TRACK_BROADPHASE_STATS and look at the stats reported on the TTY.
namespace BroadPhaseLayers
{
	static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
	static constexpr JPH::BroadPhaseLayer MOVING(1);
	static constexpr JPH::uint NUM_LAYERS(2);
};

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface
{
public:
									BPLayerInterfaceImpl()
	{
		// Create a mapping table from object to broad phase layer
		mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
		mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
	}

	virtual JPH::uint					GetNumBroadPhaseLayers() const override
	{
		return BroadPhaseLayers::NUM_LAYERS;
	}

	virtual JPH::BroadPhaseLayer			GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
	{
		JPH_ASSERT(inLayer < Layers::NUM_LAYERS);
		return mObjectToBroadPhase[inLayer];
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char *			GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
	{
		switch ((BroadPhaseLayer::Type)inLayer)
		{
		case (BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING:	return "NON_MOVING";
		case (BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:		return "MOVING";
		default:													JPH_ASSERT(false); return "INVALID";
		}
	}
#endif // JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED

private:
	JPH::BroadPhaseLayer					mObjectToBroadPhase[Layers::NUM_LAYERS];
};

/// Class that determines if an object layer can collide with a broadphase layer
class ObjectVsBroadPhaseLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
	virtual bool				ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override
	{
		switch (inLayer1)
		{
		case Layers::NON_MOVING:
			return inLayer2 == BroadPhaseLayers::MOVING;
		case Layers::MOVING:
			return true;
		default:
			JPH_ASSERT(false);
			return false;
		}
	}
};

// An example contact listener
class MyContactListener : public JPH::ContactListener
{
public:
	// See: ContactListener
	virtual JPH::ValidateResult	OnContactValidate(const JPH::Body &inBody1, const JPH::Body &inBody2, JPH::RVec3Arg inBaseOffset, const JPH::CollideShapeResult &inCollisionResult) override
	{
		cout << "Contact validate callback" << endl;

		// Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
		return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
	}

	virtual void			OnContactAdded(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override
	{
		cout << "A contact was added" << endl;
	}

	virtual void			OnContactPersisted(const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings) override
	{
		cout << "A contact was persisted" << endl;
	}

	virtual void			OnContactRemoved(const JPH::SubShapeIDPair &inSubShapePair) override
	{
		cout << "A contact was removed" << endl;
	}
};

// An example activation listener
class MyBodyActivationListener : public JPH::BodyActivationListener
{
public:
	virtual void		OnBodyActivated(const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData) override
	{
		cout << "A body got activated" << endl;
	}

	virtual void		OnBodyDeactivated(const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData) override
	{
		cout << "A body went to sleep" << endl;
	}
};

Vector3 operator*(const float& v, const Vector3& vr)
{
	return {vr.x * v, vr.y * v, vr.z * v};
}

Vector3 toRayVec(JPH::Vec3 v)
{
	return {v.GetX(), v.GetY(), v.GetZ()};
}


struct Element
{
	JPH::BodyID id;
	bool hasModel = false;
	int type = 0;
	optional<Model> model;
	Vector3 size;
};

enum TYPE
{
	SPHERE,
	RECTANGLE,
	FLOOR,
	CUSTOM
};

void addSphere(vector<Element>& elements, JPH::BodyInterface& bt, JPH::Vec3 pos, float rad)
{
	Element e;
	JPH::BodyCreationSettings sphereSettings(new JPH::SphereShape(rad), pos, JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
	e.id = bt.CreateAndAddBody(sphereSettings, JPH::EActivation::Activate);
	e.type = SPHERE;
	elements.push_back(e);
}

void addRect(vector<Element>& elements, JPH::BodyInterface& bt, JPH::Vec3 pos, JPH::Vec3 dim)
{
	Element e;
	JPH::BodyCreationSettings rectSettings(new JPH::BoxShape(dim), pos, JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
	e.id = bt.CreateAndAddBody(rectSettings, JPH::EActivation::Activate);
	e.type = RECTANGLE;
	e.size = 2.f*toRayVec(dim);
	elements.push_back(e);

}


void addFloor(vector<Element>& elements, JPH::BodyInterface& bt, JPH::Vec3 pos, JPH::Vec3 dim)
{
	Element e;
	JPH::BoxShapeSettings floor_shape_settings(dim);
	floor_shape_settings.SetEmbedded(); // A ref counted object on the stack (base class RefTarget) should be marked as such to prevent it from being freed when its reference count goes to 0.
	JPH::ShapeSettings::ShapeResult floor_shape_result = floor_shape_settings.Create();
	JPH::ShapeRefC floor_shape = floor_shape_result.Get(); // We don't expect an error here, but you can check floor_shape_result for HasError() / GetError()
	JPH::BodyCreationSettings floor_settings(floor_shape, pos, JPH::Quat::sIdentity(), JPH::EMotionType::Static, Layers::NON_MOVING);
	JPH::Body *floor = bt.CreateBody(floor_settings); // Note that if we run out of bodies this can return nullptr
	bt.AddBody(floor->GetID(), JPH::EActivation::DontActivate);
	e.id = floor->GetID();
	e.type = FLOOR;
	e.size = 2.f * toRayVec(dim);
	elements.push_back(e);
}

void draw(Element& e, JPH::BodyInterface& bi, Texture&)
{

	rlPushMatrix();
	JPH::Quat rot = bi.GetRotation(e.id);
	float angle = 0;
	JPH::Vec3 axis;
	rot.GetAxisAngle(axis, angle); 
	JPH::Vec3 pos = bi.GetPosition(e.id);
	rlTranslatef(pos.GetX(), pos.GetY(), pos.GetZ());
	rlRotatef(angle * (180/PI), axis.GetX(), axis.GetY(), axis.GetZ());
	switch(e.type)
	{
		case(SPHERE):
			DrawSphere({0, 0, 0}, bi.GetShape(e.id)->GetInnerRadius(), BLUE);
		break;
		case(RECTANGLE):
		{
			float v = (0);
			Vector3 corner = {e.size.x * v, e.size.y * v, e.size.z * v};
			DrawCube(corner, e.size.x, e.size.y, e.size.z, GREEN);
			
		}
		break;
		case(FLOOR):
			float v = (0);
			Vector3 corner = {e.size.x * v, e.size.y * v, e.size.z * v};
			DrawCube(corner, e.size.x, e.size.y, e.size.z, RED);
		break;
	}
	rlPopMatrix();
}

// Program entry point
int main(int argc, char** argv)
{

	vector<Element> elements;	
	JPH::RegisterDefaultAllocator();
	JPH::Trace = TraceImpl;
	JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)
	JPH::Factory::sInstance = new JPH::Factory();
	JPH::RegisterTypes();
	JPH::TempAllocatorImpl temp_allocator(10 * 1024 * 1024);
	JPH::JobSystemThreadPool job_system(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, thread::hardware_concurrency() - 1);
	const JPH::uint cMaxBodies = 1024;
	const JPH::uint cNumBodyMutexes = 0;
	const JPH::uint cMaxBodyPairs = 1024;
	const JPH::uint cMaxContactConstraints = 1024;
	BPLayerInterfaceImpl broad_phase_layer_interface;
	ObjectVsBroadPhaseLayerFilterImpl object_vs_broadphase_layer_filter;
	ObjectLayerPairFilterImpl object_vs_object_layer_filter;
	JPH::PhysicsSystem physics_system;
	physics_system.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, broad_phase_layer_interface, object_vs_broadphase_layer_filter, object_vs_object_layer_filter);
	MyBodyActivationListener body_activation_listener;
	physics_system.SetBodyActivationListener(&body_activation_listener);
	MyContactListener contact_listener;
	physics_system.SetContactListener(&contact_listener);
	JPH::BodyInterface &body_interface = physics_system.GetBodyInterface();

	const float cDeltaTime = 1.0f / 60.0f;
	physics_system.OptimizeBroadPhase();
	JPH::uint step = 0;
	InitWindow(512, 512, "titre");
	SetTargetFPS(60);
	Camera3D camera;
	camera.fovy = 90;
	camera.projection = CAMERA_PERSPECTIVE;
	camera.position = {-20, 10, 0};
	camera.up = {0, 1, 0};
	camera.target = {0, 0, 0};
	Image img = GenImageChecked(2, 2, 1, 1, ORANGE, BLUE);
	Texture t = LoadTextureFromImage(img);
	UnloadImage(img);

	//addSphere(elements, body_interface, {0, 10, 0}, 3.f);
	addFloor(elements, body_interface, {-10, 0, -10}, {30, 2, 30});
	addRect(elements, body_interface, {1, 15, 0}, {4, 1, 3});
	addRect(elements, body_interface, {0, 10, 0}, {1, 3, 1});
	addSphere(elements, body_interface, {0, 20, 0}, 2.f);

	DisableCursor();
	while (!WindowShouldClose())
	{
		++step;
		
		// If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
		const int cCollisionSteps = 1;

		// Step the world
		physics_system.Update(cDeltaTime, cCollisionSteps, &temp_allocator, &job_system);
		UpdateCamera(&camera, CAMERA_FREE);
		BeginDrawing();
			ClearBackground(Color(0, 0, 0));
			BeginMode3D(camera);
				for(auto& e : elements)
					draw(e, body_interface, t);
			EndMode3D();
		EndDrawing();
	}

	for(auto& e: elements)
	{
		body_interface.RemoveBody(e.id);
		body_interface.DestroyBody(e.id);
	}


	// Unregisters all types with the factory and cleans up the default material
	JPH::UnregisterTypes();
	UnloadTexture(t);
	// Destroy the factory
	delete JPH::Factory::sInstance;
	JPH::Factory::sInstance = nullptr;

	return 0;
}