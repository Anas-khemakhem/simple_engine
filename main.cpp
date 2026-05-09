#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
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
    camera.position = Vector3{ 0.0f, 2.0f, 80.0f }; // Human eye level on the laboratory walkway
    camera.target = Vector3{ 0.0f, 2.0f, 0.0f };    
    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };         
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    DisableCursor(); 

    PhysicsWorld world;
    world.gravity = {0.0f, -40.0f};

    // Static bounds (Virtual Simulation Chamber)
    world.AddBody(RigidBody::CreateAABB(Vec2{0, -5}, Vec2{80.0f, 5.0f}, 0.0f, 0.5f));
    world.AddBody(RigidBody::CreateAABB(Vec2{-60, 20}, Vec2{5.0f, 50.0f}, 0.0f, 0.5f));
    world.AddBody(RigidBody::CreateAABB(Vec2{60, 20}, Vec2{5.0f, 50.0f}, 0.0f, 0.5f));

    float timeAccumulator = 0.0f;
    const float fixedTimeStep = 1.0f / 120.0f; // High frequency for stability

    // Simulator user-controlled parameters
    float injectSpeed = 150.0f;
    float injectSpin = 0.0f;   // 0 = Normal Ball (no lift), > 0 = Magnus Effect
    float injectRadius = 2.5f;
    float injectMass = 15.0f;
    float targetGhostTime = 2.0f; // Predict where it will be at this exact future time

    while (!WindowShouldClose()) {
        // Handle input for simulator parameters
        if (IsKeyDown(KEY_UP)) injectSpeed += 100.0f * GetFrameTime();
        if (IsKeyDown(KEY_DOWN)) injectSpeed -= 100.0f * GetFrameTime();
        if (IsKeyDown(KEY_RIGHT)) injectSpin += 100.0f * GetFrameTime();
        if (IsKeyDown(KEY_LEFT)) injectSpin -= 100.0f * GetFrameTime();
        
        if (IsKeyDown(KEY_EQUAL) || IsKeyDown(KEY_KP_ADD)) injectRadius += 2.0f * GetFrameTime();
        if (IsKeyDown(KEY_MINUS) || IsKeyDown(KEY_KP_SUBTRACT)) injectRadius -= 2.0f * GetFrameTime();
        if (injectRadius < 0.5f) injectRadius = 0.5f;

        if (IsKeyDown(KEY_T)) targetGhostTime -= 2.0f * GetFrameTime();
        if (IsKeyDown(KEY_Y)) targetGhostTime += 2.0f * GetFrameTime();
        if (targetGhostTime < 0.1f) targetGhostTime = 0.1f;
        if (targetGhostTime > 15.0f) targetGhostTime = 15.0f;
        
        // Quick presets
        if (IsKeyPressed(KEY_ONE)) { injectSpin = 0.0f; injectRadius = 2.5f; } // Normal Ball
        if (IsKeyPressed(KEY_TWO)) { injectSpin = 200.0f; injectRadius = 2.5f; } // Curveball / Magnus Component
        if (IsKeyPressed(KEY_THREE)) { injectSpin = 0.0f; injectRadius = 8.0f; } // Massive Drag (Parachute effect)

        UpdateCamera(&camera, CAMERA_FIRST_PERSON); // Smooth FPS-style exploration

        // Constrain camera height to simulate walking on the laboratory floor
        if (camera.position.y < 2.0f) camera.position.y = 2.0f;
        if (camera.position.y > 2.0f) camera.position.y -= 10.0f * GetFrameTime(); // Gentle gravity for observer
        
        Vector3 mTarget = Vector3Subtract(camera.target, camera.position);
        Vector3 fwd = Vector3Normalize(mTarget);
        float t_val = (0.0f - camera.position.z) / (fwd.z != 0 ? fwd.z : 0.001f);
        Vector3 hitPoint = Vector3Add(camera.position, Vector3Scale(fwd, t_val));

        if (IsKeyPressed(KEY_TWO)) { 
            // Pressing '2' (or 'é' on AZERTY) launches a spinning ball directly
            size_t bId = world.AddBody(RigidBody::CreateCircle(Vec2{hitPoint.x, hitPoint.y}, injectRadius, injectMass, 0.4f));
            world.bodies[bId].velocity = Vec2{fwd.x, fwd.y} * injectSpeed; 
            world.bodies[bId].angular_velocity = 200.0f; 
        }

        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            // Normal kinematic particle injection (No Spin)
            size_t bId = world.AddBody(RigidBody::CreateCircle(Vec2{hitPoint.x, hitPoint.y}, injectRadius, injectMass, 0.4f));
            world.bodies[bId].velocity = Vec2{fwd.x, fwd.y} * injectSpeed; 
            world.bodies[bId].angular_velocity = 0.0f; 
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
        ClearBackground((Color){ 10, 15, 22, 255 }); // Deep dark blue background for focus
        BeginMode3D(camera);

        // Draw Laboratory Floor
        DrawGrid(100, 10.0f); // 3D Walkway grid
        
        // Draw 2D Simulation Plane (Z=0 Reference Grid)
        rlPushMatrix();
            rlRotatef(90.0f, 1.0f, 0.0f, 0.0f); // Rotate grid to stand vertically
            DrawGrid(100, 5.0f);
        rlPopMatrix();

        // -------------------------
        // MATH & SIMULATOR FEATURE: TRAJECTORY PREDICTOR
        // Calculate Numerical Integration via Symplectic Euler Predictor to show projectile path
        // It demonstrates mathematical modeling of aerodynamic forces using the same solver parameters
        // -------------------------
        Vec2 predPos = {hitPoint.x, hitPoint.y};
        Vec2 predVel = Vec2{fwd.x, fwd.y} * injectSpeed;
        float predAngVel = injectSpin;
        
        // Find out exactly how many integration steps are needed to reach the target ghost time
        int predictionSteps = (int)(targetGhostTime / fixedTimeStep);
        Vec2 ghostPosition = predPos;

        rlSetLineWidth(2.0f);
        BeginBlendMode(BLEND_ADDITIVE);
        // Compute path exactly up to the target time
        for (int k = 0; k < predictionSteps; ++k) {
            Vec2 oldPos = predPos;
            
            float vSq = predVel.LengthSq();
            Vec2 predForce = {0,0};
            if (vSq > 0.001f) {
                float v = std::sqrt(vSq);
                float area = 3.14159f * injectRadius * injectRadius;
                
                // Rayleigh Drag formulation predictor: (F_d = -0.5 * p * v^2 * Cd * A)
                float dragMag = 0.5f * 1.225f * vSq * 0.47f * area;
                dragMag = std::min(dragMag, injectMass * 1000.0f); 
                predForce -= (predVel / v) * dragMag;
                
                // Magnus Effect Lift Tensor
                if (std::abs(predAngVel) > 0.1f) {
                    float spinRatio = (injectRadius * predAngVel) / v;
                    float cl = std::min(std::max(spinRatio, -1.5f), 1.5f);
                    float liftMag = 0.5f * 1.225f * vSq * cl * area;
                    predForce += Vec2{(predVel.y/v), -(predVel.x/v)} * liftMag; // Matches PhysicsEngine Cross() direction
                }
            }
            
            // Advance prediction state
            predAngVel -= predAngVel * 0.1f * fixedTimeStep; // Dampen prediction spin accurately
            predVel += (world.gravity + predForce * (1.0f / injectMass)) * fixedTimeStep;
            
            // Apply identical engine speed limit to prediction
            if (predVel.LengthSq() > 300.0f * 300.0f) {
                predVel = predVel / predVel.Length() * 300.0f;
            }
            
            predPos += predVel * fixedTimeStep;
            
            // MATH: Perfect collision prediction matching PhysicsEngine logic
            RigidBody ghostBody = RigidBody::CreateCircle(predPos, injectRadius, injectMass, 0.4f);
            ghostBody.velocity = predVel;
            ghostBody.angular_velocity = predAngVel;
            
            for (auto& b : world.bodies) {
                Manifold m;
                if (b.shape == ShapeType::AABB) {
                    m = CheckCircleVsAABB(&ghostBody, &b);
                } else if (b.shape == ShapeType::CIRCLE) {
                    m = CheckCircleVsCircle(&ghostBody, &b);
                }
                
                if (m.colliding) {
                    // Resolve penetration directly (move AWAY from collision)
                    ghostBody.position -= m.normal * m.penetration;
                    
                    // Simple inelastic bounce matching the Engine's exact restitution and physics
                    Vec2 refA = m.contact - ghostBody.position;
                    Vec2 velA = ghostBody.velocity + Cross(ghostBody.angular_velocity, refA);
                    
                    // We treat the environment and other balls as static for the sake of the trajectory line 
                    Vec2 rv = velA * (-1.0f); // b.velocity is 0 from the predictor's perspective

                    float contactVel = rv.Dot(m.normal);
                    if (contactVel > 0) continue;

                    float e = std::min(0.4f, b.restitution); // bounce factor
                    
                    float raCrossN = Cross(refA, m.normal);
                    float invMassSum = ghostBody.inv_mass + (raCrossN * raCrossN) * ghostBody.inv_inertia;
                    
                    float j = -(1.0f + e) * contactVel / invMassSum;
                    Vec2 impulse = m.normal * j;
                    
                    // Apply Normal Impulse
                    ghostBody.velocity -= impulse * ghostBody.inv_mass;
                    ghostBody.angular_velocity -= ghostBody.inv_inertia * Cross(refA, impulse);
                    
                    // Friction prediction
                    Vec2 tangent = rv - (m.normal * rv.Dot(m.normal));
                    if (tangent.LengthSq() > 0.0001f) {
                        tangent = tangent / tangent.Length();
                        float raCrossT = Cross(refA, tangent);
                        float invMassFrictionSum = ghostBody.inv_mass + (raCrossT * raCrossT) * ghostBody.inv_inertia;
                        
                        float jt = -rv.Dot(tangent) / invMassFrictionSum;
                        float mu = std::sqrt(ghostBody.friction * b.friction);
                        
                        Vec2 frictionImpulse;
                        if (std::abs(jt) < j * mu) {
                            frictionImpulse = tangent * jt;
                        } else {
                            frictionImpulse = tangent * (j * mu * (jt > 0 ? 1.0f : -1.0f));
                        }
                        ghostBody.velocity -= frictionImpulse * ghostBody.inv_mass;
                        ghostBody.angular_velocity -= ghostBody.inv_inertia * Cross(refA, frictionImpulse);
                    }
                }
            }
            
            predPos = ghostBody.position;
            predVel = ghostBody.velocity;
            predAngVel = ghostBody.angular_velocity;
            
            // Render Prediction Vector Arc (Fade out progressively)
            DrawLine3D({oldPos.x, oldPos.y, 0.0f}, {predPos.x, predPos.y, 0.0f}, Fade(GREEN, 1.0f - (float)k/predictionSteps));
            
            if (k == predictionSteps - 1) {
                ghostPosition = predPos;
            }
        }
        EndBlendMode();
        rlSetLineWidth(1.0f);

        // Draw the predicted future location (Ghost Shadow)
        DrawSphere({ghostPosition.x, ghostPosition.y, 0.0f}, injectRadius, Fade(GREEN, 0.6f));
        DrawSphereWires({ghostPosition.x, ghostPosition.y, 0.0f}, injectRadius, 8, 8, DARKGREEN);

        // Draw joints (Kinematic Constraints)
        for (const auto& j : world.joints) {
            Vector3 pA = {world.bodies[j.bodyA].position.x, world.bodies[j.bodyA].position.y, 0.0f};
            Vector3 pB = {world.bodies[j.bodyB].position.x, world.bodies[j.bodyB].position.y, 0.0f};
            DrawLine3D(pA, pB, DARKGRAY);
        }

        for (const auto& body : world.bodies) {
            if (body.shape == ShapeType::CIRCLE) {
                Vector3 pos = { body.position.x, body.position.y, 0.0f };
                // Keep it clean
                DrawSphere(pos, body.radius, Fade(RED, 0.8f));
                
                // Standard wires
                Vector3 endPos = { 
                    body.position.x + std::cos(body.angle) * body.radius, 
                    body.position.y + std::sin(body.angle) * body.radius, 
                    0.0f 
                };
                DrawSphereWires(pos, body.radius, 8, 8, BLACK); 
                DrawLine3D(pos, endPos, BLACK); 
            } 
            else if (body.shape == ShapeType::AABB) {
                Vector3 pos = { body.position.x, body.position.y, 0.0f };
                float w = body.half_size.x * 2.0f;
                float h = body.half_size.y * 2.0f;
                DrawCube(pos, w, h, 2.0f, Fade(DARKGRAY, 0.9f)); 
                DrawCubeWires(pos, w, h, 2.0f, BLACK);
            }
        }

        EndMode3D();
        
        // Heads Up Display (HUD) for researcher
        DrawRectangle(0, 0, GetScreenWidth(), 170, Fade(BLACK, 0.8f));
        
        DrawText("AERODYNAMIC PHYSICS SIMULATOR v4.0 (Ghost Prediction)", 15, 10, 20, GREEN);
        
        // Flight parameters info panel
        DrawText(TextFormat("Injection Velocity:   %.1f m/s", injectSpeed), 400, 15, 15, YELLOW);
        DrawText(TextFormat("Injection Spin rate:  %.1f rad/s", injectSpin), 400, 35, 15, ORANGE);
        DrawText(TextFormat("Particle Radius:      %.2f m", injectRadius), 400, 55, 15, VIOLET);
        DrawText(TextFormat("Particle Mass:        %.1f kg", injectMass), 400, 75, 15, LIGHTGRAY);
        DrawText(TextFormat("Prediction Time:      %.1f sec", targetGhostTime), 400, 95, 15, LIME);

        // Control binds
        DrawText("HOTKEYS:", 15, 40, 14, SKYBLUE);
        DrawText("[1] Preset: Standard Ball (No lift)", 15, 60, 14, LIGHTGRAY);
        DrawText("[2] Preset: High Spin Curveball (Magnus effect)", 15, 75, 14, LIGHTGRAY);
        DrawText("[3] Preset: Giant Parachute (High drag surface area)", 15, 90, 14, LIGHTGRAY);
        
        DrawText("[T / Y]     Adjust Target Ghost Time", 15, 115, 12, GREEN);
        DrawText("[UP/DOWN]   Adjust Speed", 15, 130, 12, GRAY);
        DrawText("[LEFT/RIGHT] Adjust Spin", 15, 145, 12, GRAY);
        
        DrawText(TextFormat("Metrics: %d Bodies | %d Joints", (int)world.bodies.size(), (int)world.joints.size()), 400, 125, 14, DARKGREEN);
        
        DrawText(TextFormat("Metrics: %d Bodies | %d Joints", (int)world.bodies.size(), (int)world.joints.size()), 400, 125, 14, (Color){ 0, 255, 150, 255 });

        // Crosshair
        DrawCircleLines(GetScreenWidth()/2, GetScreenHeight()/2, 4.0f, Fade((Color){0, 255, 150, 255}, 0.5f));
        DrawPixel(GetScreenWidth()/2, GetScreenHeight()/2, (Color){0, 255, 150, 255});

        EndDrawing();
    }
    CloseWindow();
    return 0;
}