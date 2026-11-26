#include "physics/Collision.hpp"

namespace impulse {

bool collideCircles(const Body& a, const Body& b, Manifold& m) {
    float ra = std::get<Circle>(a.shape).radius;
    float rb = std::get<Circle>(b.shape).radius;

    Vec2 d = b.position - a.position;
    float distSq = lengthSq(d);
    float rsum = ra + rb;
    if (distSq > rsum * rsum) return false;

    float dist = std::sqrt(distSq);
    m.count = 1;
    if (dist > 1e-6f) {
        m.normal = d / dist;
    } else {
        // centers on top of each other, any direction works
        m.normal = {0.0f, 1.0f};
    }
    m.contacts[0].penetration = rsum - dist;
    // put the contact halfway through the overlap
    m.contacts[0].position = a.position + m.normal * (ra - 0.5f * m.contacts[0].penetration);
    return true;
}

bool collide(const Body& a, const Body& b, Manifold& m) {
    // only circles for now
    return collideCircles(a, b, m);
}

} // namespace impulse
