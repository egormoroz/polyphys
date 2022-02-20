#include "world.hpp"
#include "body.hpp"

const Vec2 GRAVITY(0.f, 70.f);

static void integrate_forces(Body *b, float dt) {
    if (b->inv_mass == 0)
        return;
    b->vel += (b->force * b->inv_mass + GRAVITY) * dt / 2.f;
    b->angular_vel += b->torque * b->inv_inert * dt / 2.f;
}

static void integrate_velocity(Body *b, float dt) {
    if (b->inv_mass == 0)
        return;
    b->pos += b->vel * dt;
    b->angle += b->angular_vel * dt;
    integrate_forces(b, dt);
}

void World::update() {
    int num_bodies = static_cast<int>(bodies.size());
    contacts.clear();
    for (int i = 0; i < num_bodies; ++i) {
        Body *a = bodies[i];
        for (int j = i + 1; j < num_bodies; ++j) {
            Body *b = bodies[j];
            if (a->inv_mass == 0 && b->inv_mass == 0)
                continue;

            Manifold m(a, b);
            m.solve();
            if (m.num_contacts) 
                contacts.push_back(m);
        }
    }

    for (auto &b: bodies)
        integrate_forces(b, time_step);

    for (auto &i: contacts)
        i.initialize();

    for (int i = 0; i < iterations; ++i)
        for (auto &j: contacts)
            j.apply_impulse();

    for (auto &b: bodies)
        integrate_velocity(b, time_step);

    for (auto &i: contacts)
        i.positional_correction();

    for (auto &b: bodies) {
        b->force = Vec2();
        b->torque = 0.f;
    }
}

World::~World() {
    for (auto &b: bodies)
        delete b;
}

