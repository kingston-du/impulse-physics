#pragma once

#include "physics/Body.hpp"
#include "physics/Manifold.hpp"

namespace impulse {

// Narrow phase. Fills normal and contact points and returns true if the
// shapes overlap or touch. Does not set m.a / m.b, the caller does that.
bool collide(const Body& a, const Body& b, Manifold& m);

bool collideCircles(const Body& a, const Body& b, Manifold& m);

} // namespace impulse
