#pragma once

#include "physics/World.hpp"

namespace impulse {

// Preset demo scenes, picked with the number keys. Some scenes spawn bodies
// over time (the ball rain), so there's an update() that runs every physics
// step.
class Scenes {
public:
    explicit Scenes(Vec2 worldSize) : size_(worldSize) {}

    void load(int id, World& world);
    void update(World& world);

    int current() const { return current_; }
    const char* name() const;

private:
    Vec2 size_;
    int current_ = 2;
    int toSpawn_ = 0;
    unsigned seed_ = 1;

    float rand(float lo, float hi);
    void resetWorld(World& world);
};

} // namespace impulse
