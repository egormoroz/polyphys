#ifndef POLYGON_HPP
#define POLYGON_HPP

#include "mathutils.hpp"
#include <cassert>

const int MAX_VERTICES = 16;

struct Polygon {
    Polygon() = default;

    Vec2 vertices[MAX_VERTICES];
    int num_vertices = 0;

    Vec2 get_normal(int face_idx) const {
        assert(num_vertices >= 2 && face_idx < num_vertices);
        Vec2 v = vertices[(face_idx + 1) % num_vertices]
            - vertices[face_idx];
        Vec2 n(-v.y, v.x);
        n.normalize();

        return n;
    }
};

#endif
