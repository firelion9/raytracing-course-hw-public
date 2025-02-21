#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <variant>

namespace geometry {
#include "generated/vectors.generated.inline.h"

[[nodiscard]] constexpr inline vec3 crs(const vec3 &a, const vec3 &b) {
    return {
        a.y() * b.z() - a.z() * b.y(),
        a.z() * b.x() - a.x() * b.z(),
        a.x() * b.y() - a.y() * b.x(),
    };
}

template <class vec_type>
[[nodiscard]] constexpr inline vec_type norm(const vec_type &vec) {
    return vec / vec.len();
}

template <class vec_type>
[[nodiscard]] constexpr vec_type reflect(const vec_type &normal,
                                         const vec_type &in_dir) {
    return in_dir - 2 * normal * dot(in_dir, normal);
}

struct quaternion;
[[nodiscard]] constexpr inline quaternion operator/(const quaternion &a,
                                                    float scl);

struct quaternion {
    vec3 vp;
    float sp;

    [[nodiscard]] constexpr inline quaternion(const quaternion &other) =
        default;

    [[nodiscard]] constexpr explicit inline quaternion(const vec3 &v,
                                                       float w = 0)
        : vp(v), sp(w) {}

    [[nodiscard]] constexpr explicit inline quaternion(const vec4 &value)
        : vp(value.xyz()), sp(value.w()) {}

    [[nodiscard]] constexpr explicit inline quaternion(float a = 1, float b = 0,
                                                       float c = 0, float d = 0)
        : vp{b, c, d}, sp(a) {}

    [[nodiscard]] constexpr float a() const { return sp; }

    [[nodiscard]] constexpr float b() const { return vp.x(); }

    [[nodiscard]] constexpr float c() const { return vp.y(); }

    [[nodiscard]] constexpr float d() const { return vp.z(); }

    [[nodiscard]] constexpr float &a() { return sp; }

    [[nodiscard]] constexpr float &b() { return vp.x(); }

    [[nodiscard]] constexpr float &c() { return vp.y(); }

    [[nodiscard]] constexpr float &d() { return vp.z(); }

    [[nodiscard]] constexpr float w() const { return sp; }

    [[nodiscard]] constexpr vec3 v() const { return vp; }

    [[nodiscard]] constexpr float &w() { return sp; }

    [[nodiscard]] constexpr vec3 &v() { return vp; }

    [[nodiscard]] constexpr quaternion conj() const {
        return quaternion{-v(), w()};
    }

    [[nodiscard]] constexpr quaternion inv() const {
        return conj() / std::sqrt(v().len2() + w() * w());
    }
};

[[nodiscard]] constexpr inline quaternion operator+(const quaternion &a,
                                                    const quaternion &b) {
    return quaternion(a.v() + b.v(), a.w() + b.w());
}

[[nodiscard]] constexpr inline quaternion operator-(const quaternion &a,
                                                    const quaternion &b) {
    return quaternion(a.v() - b.v(), a.w() - b.w());
}

[[nodiscard]] constexpr inline quaternion operator*(const quaternion &a,
                                                    float scl) {
    return quaternion(a.v() * scl, a.w() * scl);
}

[[nodiscard]] constexpr inline quaternion operator*(float scl,
                                                    const quaternion &a) {
    return quaternion(scl * a.v() * scl, scl * a.w());
}

[[nodiscard]] constexpr inline quaternion operator/(const quaternion &a,
                                                    float scl) {
    return quaternion(a.v() / scl, a.w() / scl);
}

[[nodiscard]] constexpr inline quaternion operator/(float scl,
                                                    const quaternion &a) {
    return quaternion(scl / a.v(), scl / a.w());
}

[[nodiscard]] constexpr inline quaternion operator*(const quaternion &a,
                                                    const quaternion &b) {
    return quaternion{a.w() * b.v() + b.w() * a.v() + crs(a.v(), b.v()),
                      a.w() * b.w() - dot(a.v(), b.v())};
}

[[nodiscard]] constexpr inline vec3 operator*(const vec3 &v,
                                              const quaternion &q) {
    return (q * quaternion(v) * q.conj()).v();
}

[[nodiscard]] constexpr inline vec3 operator*(const quaternion &q,
                                              const vec3 &v) {
    return (q * quaternion(v) * q.conj()).v();
}

inline std::istream &operator>>(std::istream &stream, quaternion &q) {
    return stream >> q.v() >> q.w();
}

struct ray {
    vec3 start;
    vec3 dir;

    [[nodiscard]] constexpr inline ray(const ray &) = default;
    [[nodiscard]] constexpr inline ray(const vec3 &start, const vec3 &dir)
        : start(start), dir(dir) {}

    [[nodiscard]] constexpr inline vec3 at(float t) const {
        return start + dir * t;
    }

    [[nodiscard]] constexpr static inline ray extend_segment(const vec3 &start,
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

    [[nodiscard]] constexpr inline vec3 normal_at(const vec3 &pos) const { return normal; }
};

struct ellipsoid {
    vec3 radius;

    [[nodiscard]] constexpr inline vec3 normal_at(const vec3 &pos) const { return norm(pos / radius); }
};

struct box {
    vec3 half_size;

    [[nodiscard]] constexpr inline vec3 normal_at(const vec3 &pos) const {
        vec3 npos = pos / half_size;
        vec3 res = npos.abs();
        auto axis = std::max_element(res.val.begin(), res.val.end());
        res = {0, 0, 0};
        *axis = std::signbit(npos.val[axis - &res.val[0]]) ? -1 : 1;
        return res;
    }
};

using shape = std::variant<plane, ellipsoid, box>;

[[nodiscard]] constexpr inline vec3 normal_at(const geometry::shape &shape,
                                              const vec3 &pos) {
    return std::visit(
        [&pos](const auto &shape) { return shape.normal_at(pos); }, shape);
}

template <class shape_type, class vec_type>
[[nodiscard]] constexpr inline geometry::ray reflect(const shape_type &shape, const geometry::ray &ray,
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

    [[nodiscard]] constexpr inline vec3 normal_at(const vec3 &pos) const {
        return rotation *
               geometry::normal_at(shape, rotation.conj() * (pos - position));
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
