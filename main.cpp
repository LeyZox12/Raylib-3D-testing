#include <iostream>
#include <vector>
#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"
#include "InitPhysics.hpp"
#include <Utils.hpp>

using namespace std;



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

void addSphere(vector<Element>& elements, JPH::BodyInterface* bt, JPH::Vec3 pos, float rad);
void addRect(vector<Element>& elements, JPH::BodyInterface* bt, JPH::Vec3 pos, JPH::Vec3 dim);
void addFloor(vector<Element>& elements, JPH::BodyInterface* bt, JPH::Vec3 pos, JPH::Vec3 dim);
void draw(Element& e, JPH::BodyInterface& bi, Texture&);

// Program entry point
int main(int argc, char** argv)
{
	init();
	Vector3 lightPos = {-10, 20, -10};
	vector<Element> elements;	
	JPH::uint step = 0;
	InitWindow(512, 512, "titre");
	SetTargetFPS(60);
	Camera3D camera;
	camera.fovy = 90;
	camera.projection = CAMERA_PERSPECTIVE;
	camera.position = {-20, 10, -20};
	camera.up = {0, 1, 0};
	camera.target = {0, 10, 0};

	Camera3D lightCamera;
	lightCamera.fovy = 110;
	lightCamera.projection = CAMERA_ORTHOGRAPHIC;
	lightCamera.position = lightPos;
	lightCamera.up = {0, 1, 0};
	lightCamera.target = {0, 5, 0};

	Image img = GenImageChecked(2, 2, 1, 1, ORANGE, BLUE);
	Texture t = LoadTextureFromImage(img);
	UnloadImage(img);

	//addSphere(elements, body_interface, {0, 10, 0}, 3.f);
	addFloor(elements, body_interface, {-10, 0, -10}, {30, 2, 30});
	addSphere(elements, body_interface, {3, 20, -5}, 2.f);
	addRect(elements, body_interface, {-5, 5, 2}, {4, 1, 3});
	addRect(elements, body_interface, {-2, 10, 0}, {1, 3, 1});

	DisableCursor();

	Shader lightShader = LoadShader("res/lightFrag.vs", "res/lightFrag.fs");

	unsigned int RESOLUTION = 1024;
	Shader shadowShader = LoadShader("res/shadow.vs", "res/shadow.fs");
	RenderTexture2D shadowMap = LoadRenderTexture(RESOLUTION, RESOLUTION);

	Matrix lightView = MatrixLookAt(lightPos, lightCamera.target, lightCamera.up);
	Matrix lightProj = MatrixOrtho(-30, 30, -30, 30, 0.1f, 100.0f);
	Matrix lightMatrix = MatrixMultiply(lightView, lightProj);

	while (!WindowShouldClose())
	{
		++step;
		
		// If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
		const int cCollisionSteps = 1;
		// Step the world
		physics_system.Update(cDeltaTime, cCollisionSteps, temp_allocator, job_system);
		UpdateCamera(&camera, CAMERA_FREE);
		BeginTextureMode(shadowMap);
			ClearBackground(WHITE);
			BeginMode3D(lightCamera);
			BeginShaderMode(shadowShader);
			SetShaderValue(shadowShader, rlGetLocationUniform(shadowShader.id, "lightPos"), &lightPos, RL_SHADER_UNIFORM_VEC3);
				for(auto& e : elements)
					draw(e, *body_interface, t);
			EndShaderMode();
			EndMode3D();
		EndTextureMode();
		BeginDrawing();
			ClearBackground(Color(0, 0, 0));
			BeginMode3D(camera);
			BeginShaderMode(shadowShader);
				
			EndShaderMode();

			BeginShaderMode(lightShader);
			SetShaderValueTexture(lightShader, rlGetLocationUniform(lightShader.id, "shadowMap"), shadowMap.texture);
			SetShaderValueMatrix(lightShader, rlGetLocationUniform(lightShader.id, "lightMatrix"), lightMatrix);
			SetShaderValue(lightShader, rlGetLocationUniform(lightShader.id, "camPos"), &camera.position, RL_SHADER_UNIFORM_VEC3);
			SetShaderValue(lightShader, rlGetLocationUniform(lightShader.id, "lightPos"), &lightPos, RL_SHADER_UNIFORM_VEC3);
				for(auto& e : elements)
					draw(e, *body_interface, t);
			EndShaderMode();
			DrawSphere(lightPos, 1.f, WHITE);
			EndMode3D();
		EndDrawing();
	}

	for(auto& e: elements)
	{
		body_interface->RemoveBody(e.id);
		body_interface->DestroyBody(e.id);
	}


	// Unregisters all types with the factory and cleans up the default material
	JPH::UnregisterTypes();
	UnloadTexture(t);
	// Destroy the factory
	delete JPH::Factory::sInstance;
	JPH::Factory::sInstance = nullptr;

	return 0;
}

void addSphere(vector<Element>& elements, JPH::BodyInterface* bt, JPH::Vec3 pos, float rad)
{
	Element e;
	JPH::BodyCreationSettings sphereSettings(new JPH::SphereShape(rad), pos, JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
	e.id = bt->CreateAndAddBody(sphereSettings, JPH::EActivation::Activate);
	e.type = SPHERE;
	elements.push_back(e);
}

void addRect(vector<Element>& elements, JPH::BodyInterface* bt, JPH::Vec3 pos, JPH::Vec3 dim)
{
	Element e;
	JPH::BodyCreationSettings rectSettings(new JPH::BoxShape(dim), pos, JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
	e.id = bt->CreateAndAddBody(rectSettings, JPH::EActivation::Activate);
	e.type = RECTANGLE;
	e.size = 2.f*toRayVec(dim);
	elements.push_back(e);

}


void addFloor(vector<Element>& elements, JPH::BodyInterface* bt, JPH::Vec3 pos, JPH::Vec3 dim)
{
	Element e;
	JPH::BoxShapeSettings floor_shape_settings(dim);
	floor_shape_settings.SetEmbedded(); // A ref counted object on the stack (base class RefTarget) should be marked as such to prevent it from being freed when its reference count goes to 0.
	JPH::ShapeSettings::ShapeResult floor_shape_result = floor_shape_settings.Create();
	JPH::ShapeRefC floor_shape = floor_shape_result.Get(); // We don't expect an error here, but you can check floor_shape_result for HasError() / GetError()
	JPH::BodyCreationSettings floor_settings(floor_shape, pos, JPH::Quat::sIdentity(), JPH::EMotionType::Static, Layers::NON_MOVING);
	JPH::Body *floor = bt->CreateBody(floor_settings); // Note that if we run out of bodies this can return nullptr
	bt->AddBody(floor->GetID(), JPH::EActivation::DontActivate);
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
