#include "physics/World.hpp"

#include <algorithm>
#include <chrono>

#include "physics/Collision.hpp"

namespace impulse {

int World::addBody(const Body& body) {
    bodies_.push_back(body);
    return static_cast<int>(bodies_.size()) - 1;
}

void World::step(float dt) {
    auto t0 = std::chrono::steady_clock::now();

    findContacts();
    integrateForces(dt);
    solveVelocities();
    integrateVelocities(dt);
    correctPositions();
    if (useBounds) enforceBounds();

    auto t1 = std::chrono::steady_clock::now();
    stats_.stepMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    stats_.contactCount = static_cast<int>(contacts_.size());
}

void World::findContacts() {
    contacts_.clear();
    stats_.pairsTested = 0;
    const int n = static_cast<int>(bodies_.size());
    // brute force O(n^2) for now
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (bodies_[i].isStatic() && bodies_[j].isStatic()) continue;
            ++stats_.pairsTested;
            Manifold m;
            if (collide(bodies_[i], bodies_[j], m)) {
                m.a = i;
                m.b = j;
                contacts_.push_back(m);
            }
        }
    }
}

void World::integrateForces(float dt) {
    for (Body& b : bodies_) {
        if (b.isStatic()) continue;
        b.velocity += (gravity + b.invMass * b.force) * dt;
        b.angularVelocity += b.invInertia * b.torque * dt;
    }
}

void World::solveVelocities() {
    // sequential impulses: go over every contact a few times so the
    // impulses can propagate through stacks and piles
    for (int it = 0; it < iterations; ++it) {
        for (const Manifold& m : contacts_) {
            resolveContact(m);
        }
    }
}

void World::resolveContact(const Manifold& m) {
    Body& A = bodies_[m.a];
    Body& B = bodies_[m.b];
    const Vec2 n = m.normal;

    for (int i = 0; i < m.count; ++i) {
        Vec2 rA = m.contacts[i].position - A.position;
        Vec2 rB = m.contacts[i].position - B.position;

        // velocity of the contact point on B relative to A
        Vec2 dv = (B.velocity + cross(B.angularVelocity, rB)) -
                  (A.velocity + cross(A.angularVelocity, rA));
        float vn = dot(dv, n);
        if (vn > 0.0f) continue; // already separating

        float e = std::min(A.restitution, B.restitution);
        if (-vn < restitutionThreshold) e = 0.0f;

        float rnA = cross(rA, n);
        float rnB = cross(rB, n);
        float k = A.invMass + B.invMass + rnA * rnA * A.invInertia + rnB * rnB * B.invInertia;
        if (k <= 0.0f) continue;

        float j = -(1.0f + e) * vn / k;
        Vec2 impulse = n * j;
        A.applyImpulse(-impulse, rA);
        B.applyImpulse(impulse, rB);
    }
}

void World::integrateVelocities(float dt) {
    // semi implicit euler, this uses the velocity that was just updated
    for (Body& b : bodies_) {
        if (b.isStatic()) continue;
        b.position += b.velocity * dt;
        b.angle += b.angularVelocity * dt;
        b.force = {};
        b.torque = 0.0f;
    }
}

void World::correctPositions() {
    for (const Manifold& m : contacts_) {
        Body& A = bodies_[m.a];
        Body& B = bodies_[m.b];
        float invMassSum = A.invMass + B.invMass;
        if (invMassSum <= 0.0f) continue;

        float pen = 0.0f;
        for (int i = 0; i < m.count; ++i) pen = std::max(pen, m.contacts[i].penetration);

        float amount = std::max(pen - penetrationSlop, 0.0f) / invMassSum * correctionPercent;
        Vec2 correction = m.normal * amount;
        // heavier bodies move less
        A.position -= correction * A.invMass;
        B.position += correction * B.invMass;
    }
}

void World::enforceBounds() {
    for (Body& b : bodies_) {
        const auto* c = std::get_if<Circle>(&b.shape);
        if (!c) continue;
        float r = c->radius;
        float e = b.restitution;
        if (b.position.x - r < boundsMin.x && b.velocity.x < 0.0f) {
            b.position.x = boundsMin.x + r;
            b.velocity.x *= -e;
        }
        if (b.position.x + r > boundsMax.x && b.velocity.x > 0.0f) {
            b.position.x = boundsMax.x - r;
            b.velocity.x *= -e;
        }
        if (b.position.y - r < boundsMin.y && b.velocity.y < 0.0f) {
            b.position.y = boundsMin.y + r;
            b.velocity.y *= -e;
        }
        if (b.position.y + r > boundsMax.y && b.velocity.y > 0.0f) {
            b.position.y = boundsMax.y - r;
            b.velocity.y *= -e;
        }
    }
}

} // namespace impulse
