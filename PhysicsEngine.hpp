#pragma once
#include <vector>
#include <cmath>
#include <algorithm>

struct Vec2 { 
    float x, y;
    Vec2(float x_ = 0, float y_ = 0) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& r) const { return {x + r.x, y + r.y}; }
    Vec2 operator-(const Vec2& r) const { return {x - r.x, y - r.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
    Vec2 operator/(float s) const { return {x / s, y / s}; }
    Vec2& operator+=(const Vec2& r) { x += r.x; y += r.y; return *this; }
    Vec2& operator-=(const Vec2& r) { x -= r.x; y -= r.y; return *this; }
    float Dot(const Vec2& r) const { return x * r.x + y * r.y; }
    float LengthSq() const { return x * x + y * y; }
    float Length() const { return std::sqrt(x * x + y * y); }
};

inline float Cross(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }
inline Vec2 Cross(float s, const Vec2& a) { return {-s * a.y, s * a.x}; }
inline Vec2 Cross(const Vec2& a, float s) { return {s * a.y, -s * a.x}; }

enum class ShapeType { CIRCLE, AABB };

struct RigidBody {
    Vec2 position;
    Vec2 velocity;
    float angle = 0;
    float angular_velocity = 0;
    float torque = 0;
    Vec2 force;
    
    float mass;
    float inv_mass;
    float inertia;
    float inv_inertia;
    
    float restitution; 
    float friction;
    ShapeType shape;
    float radius;      
    Vec2 half_size; 

    static RigidBody CreateCircle(Vec2 pos, float r, float m, float e = 0.4f) {
        RigidBody b{};
        b.position = pos; b.mass = m; b.inv_mass = (m > 0.0f) ? 1.0f / m : 0.0f;
        b.inertia = (m > 0.0f) ? 0.5f * m * r * r : 0.0f;
        b.inv_inertia = (b.inertia > 0.0f) ? 1.0f / b.inertia : 0.0f;
        b.restitution = e; b.friction = 0.5f; b.shape = ShapeType::CIRCLE; b.radius = r;
        return b;
    }

    static RigidBody CreateAABB(Vec2 pos, Vec2 half_sz, float m, float e = 0.4f) {
        RigidBody b{};
        b.position = pos; b.mass = m; b.inv_mass = (m > 0.0f) ? 1.0f / m : 0.0f;
        b.inertia = (m > 0.0f) ? (1.0f/12.0f) * m * (4.0f*(half_sz.x*half_sz.x + half_sz.y*half_sz.y)) : 0.0f;
        b.inv_inertia = (b.inertia > 0.0f) ? 1.0f / b.inertia : 0.0f;
        b.restitution = e; b.friction = 0.5f; b.shape = ShapeType::AABB; b.half_size = half_sz;
        return b;
    }
};

struct DistanceJoint {
    size_t bodyA;
    size_t bodyB;
    float resting_length;
    float stiffness;
    float damping;
};

struct Manifold {
    RigidBody* a;
    RigidBody* b;
    float penetration;
    Vec2 normal;
    Vec2 contact;
    bool colliding = false;
};

inline Manifold CheckCircleVsCircle(RigidBody* a, RigidBody* b) {
    Manifold m; m.a = a; m.b = b;
    Vec2 n = b->position - a->position;
    float r = a->radius + b->radius;
    if (n.LengthSq() >= r * r) return m;
    float d = n.Length();
    m.colliding = true;
    if (d != 0) {
        m.penetration = r - d;
        m.normal = n / d;
        m.contact = a->position + m.normal * a->radius;
    } else { 
        m.penetration = a->radius;
        m.normal = Vec2(1, 0);
        m.contact = a->position;
    }
    return m;
}

inline Manifold CheckAABBVsAABB(RigidBody* a, RigidBody* b) {
    Manifold m; m.a = a; m.b = b;
    Vec2 n = b->position - a->position;
    float x_overlap = a->half_size.x + b->half_size.x - std::abs(n.x);
    if (x_overlap > 0) {
        float y_overlap = a->half_size.y + b->half_size.y - std::abs(n.y);
        if (y_overlap > 0) {
            m.colliding = true;
            if (x_overlap < y_overlap) {
                m.normal = (n.x < 0) ? Vec2(-1, 0) : Vec2(1, 0);
                m.penetration = x_overlap;
                m.contact = (n.x < 0) ? Vec2(a->position.x - a->half_size.x, b->position.y) : Vec2(a->position.x + a->half_size.x, b->position.y);
            } else {
                m.normal = (n.y < 0) ? Vec2(0, -1) : Vec2(0, 1);
                m.penetration = y_overlap;
                m.contact = (n.y < 0) ? Vec2(b->position.x, a->position.y - a->half_size.y) : Vec2(b->position.x, a->position.y + a->half_size.y);
            }
        }
    }
    return m;
}

inline Manifold CheckCircleVsAABB(RigidBody* circle, RigidBody* aabb) {
    Manifold m; m.a = circle; m.b = aabb;
    float cX = std::max(aabb->position.x - aabb->half_size.x, std::min(circle->position.x, aabb->position.x + aabb->half_size.x));
    float cY = std::max(aabb->position.y - aabb->half_size.y, std::min(circle->position.y, aabb->position.y + aabb->half_size.y));
    Vec2 closest(cX, cY);
    Vec2 n = closest - circle->position;
    if (n.LengthSq() < circle->radius * circle->radius) {
        m.colliding = true;
        float d = n.Length();
        if (d > 0.0001f) {
            m.penetration = circle->radius - d;
            m.normal = n / d;
            m.contact = closest;
        } else {
            m.penetration = circle->radius;
            m.normal = Vec2(0, -1);
            m.contact = closest;
        }
    }
    return m;
}

class PhysicsWorld {
public:
    std::vector<RigidBody> bodies;
    std::vector<DistanceJoint> joints;
    Vec2 gravity = {0.0f, -9.81f};

    size_t AddBody(const RigidBody& body) { bodies.push_back(body); return bodies.size() - 1; }
    
    void AddJoint(size_t a, size_t b, float stiffness = 100.0f, float damping = 5.0f) {
        Vec2 delta = bodies[b].position - bodies[a].position;
        joints.push_back({a, b, delta.Length(), stiffness, damping});
    }

    void Step(float dt, int iterations = 10) {
        for (auto& b : bodies) {
            if (b.inv_mass == 0.0f) continue;
            b.velocity += (gravity + b.force * b.inv_mass) * dt;
            b.angular_velocity += (b.torque * b.inv_inertia) * dt;
            b.force = {0, 0}; b.torque = 0;
        }

        for (auto& j : joints) {
            RigidBody& a = bodies[j.bodyA];
            RigidBody& b = bodies[j.bodyB];
            Vec2 delta = b.position - a.position;
            float dist = delta.Length();
            if (dist > 0.001f) {
                Vec2 dir = delta / dist;
                float offset = dist - j.resting_length;
                Vec2 relVel = b.velocity - a.velocity;
                float force = offset * j.stiffness + relVel.Dot(dir) * j.damping;
                Vec2 fVec = dir * force;
                a.force += fVec;
                b.force -= fVec;
            }
        }

        std::vector<Manifold> manifolds;
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                RigidBody* a = &bodies[i]; RigidBody* b = &bodies[j];
                if (a->inv_mass == 0 && b->inv_mass == 0) continue;
                Manifold m;
                if (a->shape == ShapeType::CIRCLE && b->shape == ShapeType::CIRCLE) m = CheckCircleVsCircle(a, b);
                else if (a->shape == ShapeType::AABB && b->shape == ShapeType::AABB) m = CheckAABBVsAABB(a, b);
                else if (a->shape == ShapeType::CIRCLE && b->shape == ShapeType::AABB) m = CheckCircleVsAABB(a, b);
                else if (a->shape == ShapeType::AABB && b->shape == ShapeType::CIRCLE) {
                    m = CheckCircleVsAABB(b, a);
                }
                if (m.colliding) manifolds.push_back(m);
            }
        }

        for (int i = 0; i < iterations; ++i) {
            for (auto& m : manifolds) ResolveCollision(m);
        }

        for (auto& b : bodies) {
            b.position += b.velocity * dt;
            b.angle += b.angular_velocity * dt;
        }
        for (auto& m : manifolds) CorrectPosition(m);
    }

private:
    void ResolveCollision(Manifold& m) {
        Vec2 ra = m.contact - m.a->position;
        Vec2 rb = m.contact - m.b->position;
        Vec2 rv = m.b->velocity + Cross(m.b->angular_velocity, rb) - (m.a->velocity + Cross(m.a->angular_velocity, ra));
        float velAlongNormal = rv.Dot(m.normal);
        if (velAlongNormal > 0) return;

        float e = std::min(m.a->restitution, m.b->restitution);
        float raCrossN = Cross(ra, m.normal);
        float rbCrossN = Cross(rb, m.normal);
        float invMassSum = m.a->inv_mass + m.b->inv_mass + (raCrossN*raCrossN)*m.a->inv_inertia + (rbCrossN*rbCrossN)*m.b->inv_inertia;

        float j = -(1.0f + e) * velAlongNormal / invMassSum;
        Vec2 impulse = m.normal * j;
        m.a->velocity -= impulse * m.a->inv_mass;
        m.a->angular_velocity -= Cross(ra, impulse) * m.a->inv_inertia;
        m.b->velocity += impulse * m.b->inv_mass;
        m.b->angular_velocity += Cross(rb, impulse) * m.b->inv_inertia;

        // Friction
        rv = m.b->velocity + Cross(m.b->angular_velocity, rb) - (m.a->velocity + Cross(m.a->angular_velocity, ra));
        Vec2 t = rv - (m.normal * rv.Dot(m.normal));
        if(t.LengthSq() > 0.0001f) {
            t = t / t.Length();
            float raCrossT = Cross(ra, t);
            float rbCrossT = Cross(rb, t);
            float invMassSumT = m.a->inv_mass + m.b->inv_mass + (raCrossT*raCrossT)*m.a->inv_inertia + (rbCrossT*rbCrossT)*m.b->inv_inertia;
            
            float jt = -rv.Dot(t) / invMassSumT;
            float f = std::sqrt(m.a->friction * m.b->friction);
            Vec2 frictionImpulse = t * std::clamp(jt, -j * f, j * f);
            
            m.a->velocity -= frictionImpulse * m.a->inv_mass;
            m.a->angular_velocity -= Cross(ra, frictionImpulse) * m.a->inv_inertia;
            m.b->velocity += frictionImpulse * m.b->inv_mass;
            m.b->angular_velocity += Cross(rb, frictionImpulse) * m.b->inv_inertia;
        }
    }

    void CorrectPosition(Manifold& m) {
        float percent = 0.2f, slop = 0.05f;   
        float mag = std::max(m.penetration - slop, 0.0f) / (m.a->inv_mass + m.b->inv_mass) * percent;
        Vec2 correction = m.normal * mag;
        m.a->position -= correction * m.a->inv_mass;
        m.b->position += correction * m.b->inv_mass;
    }
};