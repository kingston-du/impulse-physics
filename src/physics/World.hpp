#pragma once

#include <vector>

#include "math/Vec2.hpp"
#include "physics/Body.hpp"
#include "physics/Manifold.hpp"

namespace impulse {

struct StepStats {
    double stepMs = 0.0;      // wall time of the last step()
    long long pairsTested = 0; // narrow phase calls
    int contactCount = 0;     // manifolds that actually touched
};

class World {
public:
    Vec2 gravity{0.0f, -9.81f};
    int iterations = 10;

    // closing speeds below this (m/s) get no bounce, otherwise resting
    // contacts keep hopping because gravity adds a little speed every step
    float restitutionThreshold = 0.5f;

    // linear projection to fix leftover overlap. slop lets bodies overlap a
    // tiny bit so contacts don't flicker on and off every frame.
    float penetrationSlop = 0.01f;
    float correctionPercent = 0.4f;

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
    const StepStats& stats() const { return stats_; }

private:
    void findContacts();
    void integrateForces(float dt);
    void solveVelocities();
    void resolveContact(const Manifold& m);
    void integrateVelocities(float dt);
    void correctPositions();
    void enforceBounds();

    std::vector<Body> bodies_;
    std::vector<Manifold> contacts_;
    StepStats stats_;
};

} // namespace impulse
