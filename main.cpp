#include "raylib.h"
#include "raymath.h"
#include "PhysicsEngine.hpp"

size_t CreateSoftBox(PhysicsWorld& world, float x, float y, float size) {
    float hs = size / 2.0f;
    auto tl = world.AddBody(RigidBody::CreateCircle(Vec2{x - hs, y + hs}, hs*0.5f, 1.0f, 0.1f));
    auto tr = world.AddBody(RigidBody::CreateCircle(Vec2{x + hs, y + hs}, hs*0.5f, 1.0f, 0.1f));
    auto bl = world.AddBody(RigidBody::CreateCircle(Vec2{x - hs, y - hs}, hs*0.5f, 1.0f, 0.1f));
    auto br = world.AddBody(RigidBody::CreateCircle(Vec2{x + hs, y - hs}, hs*0.5f, 1.0f, 0.1f));
    
    float stiff = 150.0f, damp = 10.0f;
    world.AddJoint(tl, tr, stiff, damp);
    world.AddJoint(bl, br, stiff, damp);
    world.AddJoint(tl, bl, stiff, damp);
    world.AddJoint(tr, br, stiff, damp);
    world.AddJoint(tl, br, stiff, damp); // Diagonal cross
    world.AddJoint(tr, bl, stiff, damp);
    return tl;
}

int main() {
    InitWindow(1280, 720, "CP Advanced Physics - Softbodies, Angular Velocity, FPS Camera");
    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = Vector3{ 0.0f, 25.0f, 80.0f }; 
    camera.target = Vector3{ 0.0f, 10.0f, 0.0f };    
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };         
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    DisableCursor(); 

    PhysicsWorld world;
    world.gravity = {0.0f, -40.0f};

    // Static bounds
    world.AddBody(RigidBody::CreateAABB(Vec2{0, -5}, Vec2{80.0f, 5.0f}, 0.0f, 0.5f));
    world.AddBody(RigidBody::CreateAABB(Vec2{-60, 20}, Vec2{5.0f, 50.0f}, 0.0f, 0.5f));
    world.AddBody(RigidBody::CreateAABB(Vec2{60, 20}, Vec2{5.0f, 50.0f}, 0.0f, 0.5f));

    // A simple pyramid of standard rigid boxes
    for (int y = 0; y < 10; ++y) {
        for (int x = 0; x < 10 - y; ++x) {
            float xPos = -18.0f + (x * 4.2f) + (y * 2.1f);
            float yPos = 2.0f + (y * 4.2f);
            world.AddBody(RigidBody::CreateAABB(Vec2{xPos, yPos}, Vec2{2.0f, 2.0f}, 1.0f, 0.3f));
        }
    }

    // A few normal bouncing balls
    world.AddBody(RigidBody::CreateCircle(Vec2{-30, 30}, 3.0f, 1.0f, 0.8f));
    world.AddBody(RigidBody::CreateCircle(Vec2{30, 40}, 4.0f, 2.0f, 0.7f));

    float timeAccumulator = 0.0f;
    const float fixedTimeStep = 1.0f / 120.0f; // High frequency for stability

    while (!WindowShouldClose()) {
        UpdateCamera(&camera, CAMERA_FREE);

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            Vector3 fwd = Vector3Subtract(camera.target, camera.position);
            fwd = Vector3Normalize(fwd);
            size_t bId = world.AddBody(RigidBody::CreateCircle(Vec2{camera.position.x, camera.position.y}, 2.5f, 10.0f, 0.6f));
            world.bodies[bId].velocity = Vec2{fwd.x, fwd.y} * 180.0f; 
        }

        // Fixed timestep for physics stability
        timeAccumulator += GetFrameTime();
        if (timeAccumulator > 0.1f) timeAccumulator = 0.1f; // Prevent spiral of death

        while (timeAccumulator >= fixedTimeStep) {
            world.Step(fixedTimeStep, 15);
            timeAccumulator -= fixedTimeStep;
            
            // Speed limits & bounds checking to prevent explosions
            for (auto& b : world.bodies) {
                if (b.velocity.LengthSq() > 300.0f * 300.0f) {
                    b.velocity = b.velocity / b.velocity.Length() * 300.0f;
                }
                if (b.position.y < -100.0f) b.position.y = -100.0f; // Don't let things fall to infinity
            }
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);

        DrawGrid(30, 10.0f);

        // Draw joints
        for (const auto& j : world.joints) {
            Vector3 pA = {world.bodies[j.bodyA].position.x, world.bodies[j.bodyA].position.y, 0.0f};
            Vector3 pB = {world.bodies[j.bodyB].position.x, world.bodies[j.bodyB].position.y, 0.0f};
            DrawLine3D(pA, pB, DARKGRAY);
        }

        for (const auto& body : world.bodies) {
            if (body.shape == ShapeType::CIRCLE) {
                Vector3 pos = { body.position.x, body.position.y, 0.0f };
                Vector3 endPos = { 
                    body.position.x + std::cos(body.angle) * body.radius, 
                    body.position.y + std::sin(body.angle) * body.radius, 
                    0.0f 
                };
                
                DrawSphere(pos, body.radius, Fade(RED, 0.8f));
                DrawSphereWires(pos, body.radius, 8, 8, BLACK); 
                DrawLine3D(pos, endPos, BLACK); 
            } 
            else if (body.shape == ShapeType::AABB) {
                Vector3 pos = { body.position.x, body.position.y, 0.0f };
                float w = body.half_size.x * 2.0f;
                float h = body.half_size.y * 2.0f;
                DrawCube(pos, w, h, 10.0f, Fade(DARKGRAY, 0.9f));
                DrawCubeWires(pos, w, h, 10.0f, BLACK);
            }
        }

        EndMode3D();
        
        DrawRectangle(0, 0, GetScreenWidth(), 90, Fade(BLACK, 0.8f));
        DrawText("RESEARCH LAB PHYSICS SIMULATION", 15, 10, 20, GREEN);
        DrawText("- WASD & Mouse to fly around the lab", 15, 35, 14, LIGHTGRAY);
        DrawText("- LEFT CLICK to fire heavy tungsten spheres", 15, 55, 14, RED);
        DrawText(TextFormat("Total Bodies: %d | Total Joints: %d", (int)world.bodies.size(), (int)world.joints.size()), 15, 75, 12, GRAY);

        EndDrawing();
    }
    CloseWindow();
    return 0;
}