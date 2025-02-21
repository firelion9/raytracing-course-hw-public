#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <variant>

namespace geometry {
#include "generated/vectors.generated.inline.h"

[[nodiscard]] inline vec3 crs(const vec3 &a, const vec3 &b) {
    return {
        a.y() * b.z() - a.z() * b.y(),
        a.z() * b.x() - a.x() * b.z(),
        a.x() * b.y() - a.y() * b.x(),
    };
}

template <class vec_type>
[[nodiscard]] inline vec_type norm(const vec_type &vec) {
    return vec / vec.len();
}

template <class vec_type>
[[nodiscard]] vec_type reflect(const vec_type &normal, const vec_type &in_dir) {
    return in_dir - 2 * normal * dot(in_dir, normal);
}

struct quaternion;
[[nodiscard]] inline quaternion operator/(const quaternion &a, float scl);

struct quaternion {
    vec4 value;

    inline quaternion(const quaternion &other) = default;

    explicit inline quaternion(const vec3 &v, float w = 0)
        : value({v.x(), v.y(), v.z(), w}) {}

    explicit inline quaternion(vec4 value) : value(std::move(value)) {}

    explicit inline quaternion(float a = 1, float b = 0, float c = 0,
                               float d = 0)
        : value({a, b, c, d}) {}

    [[nodiscard]] float a() const { return value.x(); }

    [[nodiscard]] float b() const { return value.y(); }

    [[nodiscard]] float c() const { return value.z(); }

    [[nodiscard]] float d() const { return value.w(); }

    [[nodiscard]] float &a() { return value.x(); }

    [[nodiscard]] float &b() { return value.y(); }

    [[nodiscard]] float &c() { return value.z(); }

    [[nodiscard]] float &d() { return value.w(); }

    [[nodiscard]] float w() const { return d(); }

    [[nodiscard]] vec3 v() const { return {a(), b(), c()}; }

    [[nodiscard]] quaternion conj() const { return quaternion{-v(), w()}; }

    [[nodiscard]] quaternion inv() const { return conj() / value.len(); }
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

[[nodiscard]] inline quaternion operator/(const quaternion &a, float scl) {
    return quaternion(a.value / scl);
}

[[nodiscard]] inline quaternion operator/(float scl, const quaternion &a) {
    return quaternion(scl / a.value);
}

[[nodiscard]] inline quaternion operator*(const quaternion &a,
                                          const quaternion &b) {
    return quaternion{a.w() * b.v() + b.w() * a.v() + crs(a.v(), b.v()),
                      a.w() * b.w() - dot(a.v(), b.v())};
}

[[nodiscard]] inline vec3 operator*(const vec3 &v, const quaternion &q) {
    return (q * quaternion(v) * q.conj()).v();
}

[[nodiscard]] inline vec3 operator*(const quaternion &q, const vec3 &v) {
    return (q * quaternion(v) * q.conj()).v();
}

inline std::istream &operator>>(std::istream &stream, quaternion &q) {
    return stream >> q.a() >> q.b() >> q.c() >> q.d();
}

struct ray {
    vec3 start;
    vec3 dir;

    inline ray(const ray &) = default;
    inline ray(const vec3 &start, const vec3 &dir) : start(start), dir(dir) {}

    [[nodiscard]] inline vec3 at(float t) const { return start + dir * t; }

    [[nodiscard]] static inline ray extend_segment(const vec3 &start,
                                                   const vec3 &end) {
        return ray(start, norm(end - start));
    }
};

enum ShapeType {
    PLANE = 0,
    ELLIPSOID = 1,
    BOX = 2,
};

struct plane {
    vec3 normal;

    inline vec3 normal_at(const vec3 &pos) const { return normal; }
};

struct ellipsoid {
    vec3 radius;

    inline vec3 normal_at(const vec3 &pos) const {
        return norm(pos / radius);
    }
};

struct box {
    vec3 half_size;

    inline vec3 normal_at(const vec3 &pos) const {
        vec3 npos = pos / half_size;
        vec3 res = npos.abs();
        auto axis = std::max_element(res.val.begin(), res.val.end());
        res = {0, 0, 0};
        *axis = std::signbit(npos.val[axis - &res.val[0]]) ? -1 : 1;
        return res;
    }
};

using shape = std::variant<plane, ellipsoid, box>;

[[nodiscard]] inline vec3 normal_at(const geometry::shape &shape,
                                    const vec3 &pos) {
    return std::visit(
        [&pos](const auto &shape) { return shape.normal_at(pos); }, shape);
}

template <class shape_type, class vec_type>
geometry::ray reflect(const shape_type &shape, const geometry::ray &ray,
                      float t) {
    auto base = ray.at(t);
    auto normal = shape.normal_at(base);
    auto out_dir = reflect(ray.dir, normal);

    return {base, out_dir};
}

struct diffuse {};

struct metallic {};

struct dielectric {
    float ior = 0;
};

using material = std::variant<diffuse, metallic, dielectric>;

struct Object {
    vec3 position;
    quaternion rotation = quaternion(0, 0, 0, 1);
    color3 color = {0, 0, 0};

    geometry::shape shape;
    geometry::material material;

    [[nodiscard]] inline vec3 normal_at(const vec3 &pos) const {
        return rotation * geometry::normal_at(shape, rotation.conj() * (pos - position));
    }
};

struct directed_light {
    vec3 direction;
};

struct point_light {
    vec3 position;
    vec3 attenuation;
};

using light = std::variant<directed_light, point_light>;

struct LightSource {
    color3 intensity{1, 1, 1};
    geometry::light light;
};
} // namespace geometry

#endif // GEOMETRY_H
