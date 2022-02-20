#include "collision.hpp"
#include "manifold.hpp"
#include "body.hpp"
#include <cfloat>
#include <algorithm>

Vec2 calc_support(const Polygon &p, const Vec2 &dir) {
    assert(p.num_vertices > 0);
    Vec2 best_vertex = p.vertices[0];
    float best_prj = dot(best_vertex, dir);

    for (int i = 1; i < p.num_vertices; ++i) {
        float prj = dot(p.vertices[i], dir);
        if (prj > best_prj) {
            best_vertex = p.vertices[i];
            best_prj = prj;
        }
    }

    return best_vertex;
}

float calc_penetr(const Polygon &p, const Vec2 &ppos, float pang,
        const Polygon &q, const Vec2 &qpos, float qang, int &best_idx) {
    best_idx = -1;

    Mat22 qT(-qang), prot(pang);
    float best_d = -FLT_MAX;
    for (int i = 0; i < p.num_vertices; ++i) {
        Vec2 n = qT * prot * p.get_normal(i);
        Vec2 s = calc_support(q, -n);

        Vec2 v = prot * p.vertices[i] + ppos;
        v = qT * (v - qpos);
        float d = dot(s - v, n);
        if (d > best_d) {
            best_d = d;
            best_idx = static_cast<int>(i);
        }
    }

    return best_d;
}

void find_incident_face(const Body *inc, const Body *ref,
    int ref_face_idx, Vec2 v[2])
{
    Mat22 irot(inc->angle), rrot(ref->angle);
    Vec2 ref_n = irot.transposed() * (rrot * ref->shape.get_normal(ref_face_idx));
    int face_idx = 0;
    float min_prj = FLT_MAX;

    for (int i = 0; i < inc->shape.num_vertices; ++i) {
        float prj = dot(inc->shape.get_normal(i), ref_n);
        if (prj < min_prj) {
            min_prj = prj;
            face_idx = i;
        }
    }

    v[0] = irot * inc->shape.vertices[face_idx] + inc->pos;
    v[1] = irot * inc->shape.vertices[(face_idx + 1) % inc->shape.num_vertices] + inc->pos;

}

struct ClippedPoints {
    Vec2 p[2];
    int n = 0;
};

ClippedPoints clip(const Vec2 v_in[2], Vec2 a, float off) {
    ClippedPoints cp;

    float d0 = dot(v_in[0], a) - off,
          d1 = dot(v_in[1], a) - off;

    if (d0 < 0.f)
        cp.p[cp.n++] = v_in[0];
    if (d1 < 0.f)
        cp.p[cp.n++] = v_in[1];

    if (d0 * d1 <= 0.f) {
        float k = d1 / (d1 - d0);
        cp.p[cp.n++] = v_in[1] + k * (v_in[0] - v_in[1]);
    }

    return cp;
}

ClippedPoints full_clip(const Vec2 inc[2], const Vec2 ref[2], Vec2 ref_n) {
    Vec2 ref_a = ref[1] - ref[0];

    ClippedPoints cp;
    if ((cp = clip(inc, ref_a, dot(ref[1], ref_a))).n < 2)
        return cp;
    if ((cp = clip(cp.p, -ref_a, dot(ref[0], -ref_a))).n < 2)
        return cp;

    return clip(cp.p, ref_n, dot(ref[0], ref_n));
}

void collision(Manifold &m) {
    m.num_contacts = 0;

    int adx, bdx;
    float apen = calc_penetr(m.a->shape, m.a->pos, m.a->angle,
            m.b->shape, m.b->pos, m.b->angle, adx);
    if (apen >= 0.f)
        return;

    float bpen = calc_penetr(m.b->shape, m.b->pos, m.b->angle,
            m.a->shape, m.a->pos, m.a->angle, bdx);
    if (bpen >= 0.f)
        return;

    Body *a = m.a, *b = m.b;
    if (apen < bpen) {
        std::swap(m.a, m.b);
        std::swap(a, b);
        std::swap(adx, bdx);
        std::swap(apen, bpen);
    }

    Mat22 arot(a->angle), brot(b->angle);
    Vec2 inc[2], ref[2], ref_n;
    ref[0] = arot * a->shape.vertices[adx] + a->pos;
    ref[1] = arot * a->shape.vertices[(adx + 1) % a->shape.num_vertices] + a->pos;
    find_incident_face(b, a, adx, inc);

    ref_n = arot * a->shape.get_normal(adx);
    ClippedPoints cp = full_clip(inc, ref, ref_n);
    if (cp.n < 2)
        return;

    const float EPS = 1e-4;
    float off = dot(ref[0], ref_n);
    float pen0 = off - dot(cp.p[0], ref_n);
    float pen1 = off - dot(cp.p[1], ref_n);

    m.penetration = (pen0 + pen1) / 2;
    if (abs(pen0) > EPS)
        m.contacts[m.num_contacts++] = cp.p[0];
    if (abs(pen1) > EPS)
        m.contacts[m.num_contacts++] = cp.p[1];

    m.normal = ref_n;
}

