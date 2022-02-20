#include "manifold.hpp"
#include "collision.hpp"
#include "body.hpp"

const float EPS = 1e-5;

Manifold::Manifold(Body *a, Body *b)
    : a(a), b(b) {}

void Manifold::solve() {
    collision(*this);
}

void Manifold::initialize() {
    restitution = fmin(a->restitution, b->restitution);
    stat_fric = sqrtf(a->stat_fric * b->stat_fric);
    dyn_fric = sqrtf(a->dyn_fric * b->dyn_fric);
}

void Manifold::apply_impulse()  {
    if (a->inv_mass < EPS && b->inv_mass < EPS)
        return;

    for (int i = 0; i < num_contacts; ++i) {
        Vec2 ra = contacts[i] - a->pos,
             rb = contacts[i] - b->pos;

        Vec2 rv = b->vel + cross(b->angular_vel, rb)
            - (a->vel + cross(a->angular_vel, ra));

        float contact_vel = dot(rv, normal);
        if (contact_vel > 0)
            return; //continue?

        float ra_x_n = cross(ra, normal),
              rb_x_n = cross(rb, normal);

        float denom = a->inv_mass + b->inv_mass 
            + ra_x_n * ra_x_n * a->inv_inert
            + rb_x_n * rb_x_n * b->inv_inert;

        float jn = -(1.f + restitution) * contact_vel / (denom * num_contacts);

        Vec2 impulse = normal * jn;
        a->apply_impulse(-impulse, ra);
        b->apply_impulse(impulse, rb);


        rv = b->vel + cross(b->angular_vel, rb)
            - (a->vel + cross(a->angular_vel, ra)); 

        Vec2 t = rv - dot(rv, normal) * normal;
        t.normalize();

        float jt = -dot(rv, t) / (denom * num_contacts);
        if (abs(jt) < EPS)
            return; //continue?

        if (fabs(jt) < jn * stat_fric)
            impulse = t * jt;
        else
            impulse = -jn * t * dyn_fric;

        a->apply_impulse(-impulse, ra);
        b->apply_impulse(impulse, rb);
    }
}

void Manifold::positional_correction() {
    const float SLOP = 0.05f;
    const float PERCENT = 0.4f;
    if (a->inv_mass == 0 && b->inv_mass == 0)
        return;
    Vec2 correction = fmaxf(penetration - SLOP, 0.0f) 
        / (a->inv_mass + b->inv_mass) * PERCENT * normal;
    a->pos -= a->inv_mass * correction;
    b->pos += b->inv_mass * correction;
}

