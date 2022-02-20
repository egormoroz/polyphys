#ifndef MATHUTILS_HPP
#define MATHUTILS_HPP

#include <cmath>

struct Vec2 {
    float x, y;

    explicit Vec2(float x = 0.f, float y = 0.f)
        : x(x), y(y) {}

    float norm_squared() const { return x * x + y * y; }
    float norm() const { return sqrtf(norm_squared()); }

    void normalize() {
        float d = norm();
        if (d != 0.f) {
            x /= d;
            y /= d;
        }
    }
};

inline Vec2 operator+(const Vec2 &lhs, const Vec2 &rhs) {
    return Vec2(lhs.x + rhs.x, lhs.y + rhs.y);
}

inline Vec2& operator+=(Vec2 &lhs, const Vec2 &rhs) {
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    return lhs;
}

inline Vec2 operator-(const Vec2 &lhs, const Vec2 &rhs) {
    return Vec2(lhs.x - rhs.x, lhs.y - rhs.y);
}

inline Vec2& operator-=(Vec2 &lhs, const Vec2 &rhs) {
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
    return lhs;
}

inline Vec2 operator*(const Vec2 &v, float k) {
    return Vec2(v.x * k, v.y * k);
}

inline Vec2 operator*(float k, const Vec2 &v) {
    return Vec2(v.x * k, v.y * k);
}

inline Vec2& operator*=(Vec2 &v, float k) {
    v.x *= k;
    v.y *= k;
    return v;
}

inline Vec2 operator/(const Vec2 &v, float k) {
    return Vec2(v.x / k, v.y / k);
}

inline Vec2& operator/=(Vec2 &v, float k) {
    v.x /= k;
    v.y /= k;
    return v;
}

inline Vec2 operator-(const Vec2 &v) {
    return Vec2(-v.x, -v.y);
}

inline bool operator==(const Vec2 &lhs, const Vec2 &rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
}


inline float dot(const Vec2 &lhs, const Vec2 &rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y;
}

inline Vec2 cross(const Vec2 &lhs, float k) {
    return Vec2(lhs.y * k, -lhs.x * k);
}

inline Vec2 cross(float k, const Vec2 &rhs) {
    return -cross(rhs, k);
}

inline float cross(const Vec2 &lhs, const Vec2 &rhs) {
    return lhs.x * rhs.y - rhs.x * lhs.y;
}

struct Mat22 {
    Mat22(float m00, float m01, float m10, float m11)
        : m00(m00), m10(m10), m01(m01), m11(m11) {}

    Mat22(const Vec2 &col0, const Vec2 &col1)
        : col0(col0), col1(col1) {}

    Mat22(float phi = 0.f) {
        float cos_phi = cosf(phi), sin_phi = sinf(phi);
        m00 = cos_phi; m01 = -sin_phi;
        m10 = sin_phi; m11 = cos_phi;
    }

    Mat22 transposed() const {
        return Mat22(m00, m10, m01, m11);
    }

    Vec2 row0() const { return Vec2(m00, m01); }
    Vec2 row1() const { return Vec2(m10, m11); }

    Mat22 operator*(const Mat22 &u) const {
        return Mat22(
            m00 * u.m00 + m01 * u.m10,
            m00 * u.m01 + m01 * u.m11,
            m10 * u.m00 + m11 * u.m10,
            m10 * u.m01 + m11 * u.m11
        );
    }

    Vec2 operator*(const Vec2 &v) const {
        return Vec2(dot(row0(), v), dot(row1(), v));
    }

    union {
        struct {
            float m00, m10, m01, m11;
        };
        struct {
            Vec2 col0, col1;
        };
    };
};

#endif
