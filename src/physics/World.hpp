#pragma once

#include <vector>

#include "math/Vec2.hpp"
#include "physics/Body.hpp"
#include "physics/Manifold.hpp"

namespace impulse {

class World {
public:
    Vec2 gravity{0.0f, -9.81f};

    // circles get clamped inside this box when enabled. stand in for real
    // walls until there's a static shape to collide with.
    bool useBounds = true;
    Vec2 boundsMin{0.0f, 0.0f};
    Vec2 boundsMax{64.0f, 36.0f};

    int addBody(const Body& body);
    void clear() { bodies_.clear(); }

    void step(float dt);

    std::vector<Body>& bodies() { return bodies_; }
    const std::vector<Body>& bodies() const { return bodies_; }
    const std::vector<Manifold>& contacts() const { return contacts_; }

private:
    void findContacts();
    void integrateForces(float dt);
    void integrateVelocities(float dt);
    void enforceBounds();

    std::vector<Body> bodies_;
    std::vector<Manifold> contacts_;
};

} // namespace impulse
