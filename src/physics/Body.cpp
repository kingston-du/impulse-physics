#include "physics/Body.hpp"

#include <numbers>

namespace impulse {

Body::Body(const Shape& s, Vec2 pos, float density) : position(pos), shape(s) {
    setDensity(density);
}

void Body::setDensity(float density) {
    if (const auto* c = std::get_if<Circle>(&shape)) {
        float r2 = c->radius * c->radius;
        mass = std::numbers::pi_v<float> * r2 * density;
        // solid disk: I = 1/2 m r^2
        inertia = 0.5f * mass * r2;
    }

    invMass = mass > 0.0f ? 1.0f / mass : 0.0f;
    invInertia = inertia > 0.0f ? 1.0f / inertia : 0.0f;
}

void Body::applyImpulse(Vec2 impulse, Vec2 contactVector) {
    velocity += invMass * impulse;
    angularVelocity += invInertia * cross(contactVector, impulse);
}

} // namespace impulse
