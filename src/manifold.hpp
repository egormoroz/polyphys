#ifndef MANIFOLD_HPP
#define MANIFOLD_HPP

#include "mathutils.hpp"

struct Body;

struct Manifold {
    Manifold(Body *a, Body *b);

    void solve();
    void initialize();
    void apply_impulse();
    void positional_correction();

    Vec2 contacts[2];
    int num_contacts;

    Vec2 normal;
    float penetration;
    float restitution;
    float dyn_fric;
    float stat_fric;

    Body *a, *b;
};

#endif
