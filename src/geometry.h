#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cmath>
#include <iostream>

namespace geometry {
#include "generated/vectors.generated.inline.h"

struct quaternion {
    vec4 value;

    inline quaternion(const quaternion &other) = default;
    explicit inline quaternion(vec4 value) : value(value) {}
    explicit inline quaternion(float a = 1, float b = 0, float c = 0,
                               float d = 0)
        : value({a, b, c, d}) {}

    [[nodiscard]] float a() const { return value.x; }

    [[nodiscard]] float b() const { return value.y; }

    [[nodiscard]] float c() const { return value.z; }

    [[nodiscard]] float d() const { return value.w; }
    [[nodiscard]] float &a() { return value.x; }

    [[nodiscard]] float &b() { return value.y; }

    [[nodiscard]] float &c() { return value.z; }

    [[nodiscard]] float &d() { return value.w; }

    [[nodiscard]] quaternion inv() const {
        float len = value.len();
        return quaternion{
            a() / len,
            -b() / len,
            -c() / len,
            -d() / len,
        };
    }
};

[[nodiscard]] inline quaternion operator+(const quaternion &a,
                                          const quaternion &b) {
    return quaternion(a.value + b.value);
}

[[nodiscard]] inline quaternion operator-(const quaternion &a,
                                          const quaternion &b) {
    return quaternion(a.value - b.value);
}

[[nodiscard]] inline quaternion operator*(const quaternion &a, float scl) {
    return quaternion(a.value * scl);
}

[[nodiscard]] inline quaternion operator*(float scl, const quaternion &a) {
    return quaternion(scl * a.value);
}

[[nodiscard]] inline quaternion operator*(const quaternion &a,
                                          const quaternion &b) {
    return quaternion{
        a.a() * a.a() - a.b() * b.b() - a.c() * b.c() - a.d() * b.d(),
        a.a() * a.b() + a.b() * b.a() + a.c() * b.d() - a.c() * b.d(),
        a.a() * a.c() - a.b() * b.d() + a.c() * b.a() - a.d() * b.b(),
        a.a() * a.d() + a.b() * b.c() - a.c() * b.b() + a.d() * b.a(),
    };
}

inline std::istream &operator>>(std::istream &stream, quaternion &q) {
    return stream >> q.a() >> q.b() >> q.c() >> q.d();
}

enum ShapeType {
    PLANE = 0,
    ELLIPSOID = 1,
    BOX = 2,
};

struct Shape {
    color4 color = {0, 0, 0, 1};
    quaternion rotation = quaternion(0, 0, 0, 1);
    vec3 position;

    vec3 prop;

    ShapeType type = PLANE;

    inline virtual ~Shape() = default;
};
} // namespace geometry

#endif // GEOMETRY_H
