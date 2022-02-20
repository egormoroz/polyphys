#ifndef BODY_HPP
#define BODY_HPP

#include "mathutils.hpp"
#include "polygon.hpp"


struct Body {
    Vec2 pos;
    Vec2 vel;

    float torque = 0.f;
    float angular_vel = 0.f;
    float angle = 0.f;

    Vec2 force;

    float intert = 0.1f, inv_inert = 10.f;
    float mass = 1.f, inv_mass = 1.f;

    float restitution = 0.5f;
    float stat_fric = 0.1f, dyn_fric = 0.09f;

    Polygon shape;

    void apply_force(const Vec2 &force) {
        this->force += force;
    }

    void apply_impulse(const Vec2 &impulse, const Vec2 &contact_vector) {
        vel += inv_mass * impulse;
        angular_vel += inv_inert * cross(contact_vector, impulse);
    }
};

#endif
