#ifndef WORLD_HPP
#define WORLD_HPP

#include <vector>
#include "manifold.hpp"


struct Body;

struct World {
    void update();

    std::vector<Body*> bodies;
    std::vector<Manifold> contacts;

    float time_step;
    int iterations;

    ~World();
};

#endif

