#include "app/Scenes.hpp"

namespace impulse {

float Scenes::rand(float lo, float hi) {
    // same LCG as the tests, keeps scenes repeatable
    seed_ = seed_ * 1664525u + 1013904223u;
    return lo + (hi - lo) * (static_cast<float>(seed_ >> 8) / static_cast<float>(1u << 24));
}

void Scenes::resetWorld(World& world) {
    world.clear();
    world.gravity = {0.0f, -9.81f};
    world.useBounds = true;
    world.boundsMin = {0.0f, 0.0f};
    world.boundsMax = size_;
    toSpawn_ = 0;
    seed_ = 1;
}

void Scenes::load(int id, World& world) {
    resetWorld(world);
    current_ = id;

    switch (id) {
    case 2:
        // 2000 balls, dropped in a few at a time from the top
        toSpawn_ = 2000;
        break;
    case 5: {
        // stress test: fill most of the screen with small balls at once
        const float r = 0.25f;
        const float spacing = 2.0f * r + 0.05f;
        for (float y = r + 0.1f; y < size_.y - 2.0f; y += spacing) {
            for (float x = r + 0.1f; x < size_.x - r; x += spacing) {
                Body b(Circle{r}, Vec2{x, y});
                b.velocity = {rand(-1.0f, 1.0f), rand(-1.0f, 1.0f)};
                world.addBody(b);
            }
        }
        break;
    }
    default:
        current_ = 2;
        toSpawn_ = 2000;
        break;
    }
}

void Scenes::update(World& world) {
    if (toSpawn_ <= 0) return;
    for (int i = 0; i < 8 && toSpawn_ > 0; ++i, --toSpawn_) {
        Body b(Circle{rand(0.25f, 0.4f)}, Vec2{rand(1.0f, size_.x - 1.0f), size_.y - 1.0f});
        b.velocity = {rand(-2.0f, 2.0f), rand(-6.0f, -2.0f)};
        b.restitution = 0.3f;
        world.addBody(b);
    }
}

const char* Scenes::name() const {
    switch (current_) {
    case 2: return "ball rain";
    case 5: return "stress test";
    default: return "?";
    }
}

} // namespace impulse
