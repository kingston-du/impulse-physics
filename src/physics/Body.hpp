#pragma once

#include "math/Vec2.hpp"
#include "physics/Shape.hpp"

namespace impulse {

// Units are meters, kilograms, seconds. +y is up.
struct Body {
    Vec2 position;
    Vec2 velocity;
    Vec2 force;

    float angle = 0.0f;
    float angularVelocity = 0.0f;
    float torque = 0.0f;

    float mass = 0.0f;
    float invMass = 0.0f;
    float inertia = 0.0f;
    float invInertia = 0.0f;

    float restitution = 0.2f;
    float friction = 0.4f;

    Shape shape;

    Body() = default;
    // density of 0 makes the body static (infinite mass)
    Body(const Shape& s, Vec2 pos, float density = 1.0f);

    bool isStatic() const { return invMass == 0.0f; }

    void setDensity(float density);
    void applyForce(Vec2 f) { force += f; }
    void applyImpulse(Vec2 impulse, Vec2 contactVector);
};

} // namespace impulse
