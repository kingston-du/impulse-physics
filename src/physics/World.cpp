#include "physics/World.hpp"

#include "physics/Collision.hpp"

namespace impulse {

int World::addBody(const Body& body) {
    bodies_.push_back(body);
    return static_cast<int>(bodies_.size()) - 1;
}

void World::step(float dt) {
    findContacts();
    integrateForces(dt);
    integrateVelocities(dt);
    if (useBounds) enforceBounds();
}

void World::findContacts() {
    contacts_.clear();
    const int n = static_cast<int>(bodies_.size());
    // brute force O(n^2) for now
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (bodies_[i].isStatic() && bodies_[j].isStatic()) continue;
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
