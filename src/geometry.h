#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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

[[nodiscard]] constexpr inline float det(const vec3 &c1, const vec3 &c2,
                                         const vec3 &c3) {
    return dot(c1, crs(c2, c3));
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

template <class vec_type>
[[nodiscard]] constexpr static inline float max_component(const vec_type &vec) {
    return *std::max_element(vec.val.begin(), vec.val.end());
}

template <class vec_type>
[[nodiscard]] constexpr static inline float min_component(const vec_type &vec) {
    return *std::min_element(vec.val.begin(), vec.val.end());
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

    [[nodiscard]] constexpr const vec3 &v() const { return vp; }

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
    return quaternion(scl * a.v(), scl * a.w());
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
    auto t = 2 * crs(q.v(), v);
    return v + q.w() * t + crs(q.v(), t);
}

[[nodiscard]] constexpr inline vec3 operator*(const quaternion &q,
                                              const vec3 &v) {
    return v * q;
}

inline std::istream &operator>>(std::istream &stream, quaternion &q) {
    return stream >> q.v() >> q.w();
}

struct matrix4 {
    std::array<vec4, 4> val{};

    [[nodiscard]] static constexpr inline matrix4 translation(const vec3 &pos) {
        return {{
            vec4{1, 0, 0, pos.x()},
            {0, 1, 0, pos.y()},
            {0, 0, 1, pos.z()},
            {0, 0, 0, 1},
        }};
    }

    [[nodiscard]] static constexpr inline matrix4 scale(const vec3 &scl) {
        return {{
            vec4{scl.x(), 0, 0, 0},
            {0, scl.y(), 0, 0},
            {0, 0, scl.z(), 0},
            {0, 0, 0, 1},
        }};
    }

    [[nodiscard]] static constexpr inline matrix4
    rotation(const quaternion &q) {
        auto q1 = q;
        const float w = q1.w();
        const float x = q1.v().x();
        const float y = q1.v().y();
        const float z = q1.v().z();

        return {{
            vec4{1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
                 2 * (x * z + y * w), 0},
            vec4{2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
                 2 * (y * z - x * w), 0},
            vec4{2 * (x * z - y * w), 2 * (y * z + x * w),
                 1 - 2 * (x * x + y * y), 0},
            {0, 0, 0, 1},
        }};
    }

    [[nodiscard]] static constexpr inline matrix4
    transform(const vec3 &scale, const quaternion &rotation,
              const vec3 &translation);

    [[nodiscard]] static constexpr inline matrix4 id() {
        return {{
            vec4{1, 0, 0, 0},
            {0, 1, 0, 0},
            {0, 0, 1, 0},
            {0, 0, 0, 1},
        }};
    }

    [[nodiscard]] constexpr inline vec3 apply(const vec3 &vec) const;

    [[nodiscard]] constexpr inline vec3 apply3(const vec3 &vec) const;
};

[[nodiscard]] constexpr inline matrix4 operator*(const matrix4 &a,
                                                 const matrix4 &b) {
    matrix4 res;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                res.val[i].val[k] += a.val[i].val[j] * b.val[j].val[k];
            }
        }
    }

    return res;
}

[[nodiscard]] constexpr inline vec4 operator*(const matrix4 &mat,
                                              const vec4 &vec) {
    vec4 res;
    for (int i = 0; i < 4; ++i) {
        res.val[i] = dot(mat.val[i], vec);
    }

    return res;
}

[[nodiscard]] constexpr inline vec4 operator*(const vec4 &vec,
                                              const matrix4 &mat) {
    vec4 res{0, 0, 0, 0};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            res.val[j] += vec.val[i] * mat.val[i].val[j];
        }
    }

    return res;
}

[[nodiscard]] constexpr inline matrix4
matrix4::transform(const vec3 &scale, const quaternion &rotation,
                   const vec3 &translation) {
    return matrix4::translation(translation) * matrix4::rotation(rotation) *
           matrix4::scale(scale);
}

[[nodiscard]] constexpr inline vec3 matrix4::apply(const vec3 &vec) const {
    return ((*this) * vec4(vec, 1)).xyz();
}

[[nodiscard]] constexpr inline vec3 matrix4::apply3(const vec3 &vec) const {
    return ((*this) * vec4(vec, 0)).xyz();
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

struct aabb {
    std::array<vec3, 2> vs{vec3{INFINITY, INFINITY, INFINITY},
                           vec3{-INFINITY, -INFINITY, -INFINITY}};

    [[nodiscard]] constexpr inline const vec3 &vmin() const { return vs[0]; }

    [[nodiscard]] constexpr inline const vec3 &vmax() const { return vs[1]; }

    [[nodiscard]] constexpr inline aabb extended_by(const vec3 &p) const {
        aabb res = *this;
        res.extend(p);
        return res;
    }

    constexpr inline void extend(const vec3 &p) {
        vs[0] = min(vs[0], p);
        vs[1] = max(vs[1], p);
    }

    constexpr inline void extend(const aabb &box) {
        vs[0] = min(vmin(), box.vmin());
        vs[1] = max(vmax(), box.vmax());
    }

    [[nodiscard]] constexpr inline vec3 vert(std::uint32_t idx) const {
        return {
            (idx & 1 ? vs[1] : vs[0]).val[0],
            (idx & 2 ? vs[1] : vs[0]).val[1],
            (idx & 4 ? vs[1] : vs[0]).val[2],
        };
    }

    template <class Fn> constexpr inline void foreach_vert(Fn &&fn) const {
        for (int i = 0; i < 8; ++i) {
            fn(vert(i));
        }
    }

    [[nodiscard]] constexpr inline vec3 diag() const { return vmax() - vmin(); }

    [[nodiscard]] constexpr inline float surface_area() const {
        return 2 * dot(diag(), diag().yxz());
    }

    [[nodiscard]] constexpr inline vec3 center() const { return vec3{0, 0, 0}; }

    [[nodiscard]] constexpr inline aabb bounding_box() const;
};

static constexpr aabb WHOLE_VOLUME{{vec3{-INFINITY, -INFINITY, -INFINITY},
                                    vec3{INFINITY, INFINITY, INFINITY}}};

[[nodiscard]] constexpr inline aabb aabb::bounding_box() const {
    return WHOLE_VOLUME;
}

[[nodiscard]] constexpr inline aabb operator*(const quaternion &a,
                                              const aabb &box) {
    aabb res;
    box.foreach_vert([&res, &a](const vec3 &vert) { res.extend(a * vert); });
    return res;
}

[[nodiscard]] constexpr inline aabb operator*(const aabb &box,
                                              const quaternion &a) {
    return a * box;
}

[[nodiscard]] constexpr inline aabb operator+(const vec3 &p, const aabb &box) {
    return {{
        box.vs[0] + p,
        box.vs[1] + p,
    }};
}

[[nodiscard]] constexpr inline aabb operator+(const aabb &box, const vec3 &p) {
    return p + box;
}

struct triangle {
    std::array<vec3, 3> vertices;

    [[nodiscard]] constexpr inline vec3 &a() { return vertices[0]; }

    [[nodiscard]] constexpr inline vec3 &b() { return vertices[1]; }

    [[nodiscard]] constexpr inline vec3 &c() { return vertices[2]; }

    [[nodiscard]] constexpr inline const vec3 &a() const { return vertices[0]; }

    [[nodiscard]] constexpr inline const vec3 &b() const { return vertices[1]; }

    [[nodiscard]] constexpr inline const vec3 &c() const { return vertices[2]; }

    [[nodiscard]] constexpr inline vec3 v() const { return b() - a(); }

    [[nodiscard]] constexpr inline vec3 u() const { return c() - a(); }

    [[nodiscard]] constexpr inline vec3 normal_at(const vec3 &pos) const {
        return norm(crs(v(), u()));
    }

    [[nodiscard]] constexpr inline float square() const {
        return crs(v(), u()).len() / 2;
    }

    [[nodiscard]] constexpr inline vec3 center() const {
        return (a() + b() + c()) / 3;
    }

    [[nodiscard]] constexpr inline aabb bounding_box() const {
        aabb res;
        res.extend(a());
        res.extend(b());
        res.extend(c());
        return res;
    }
};

using shape = triangle;

template <class shape_type, class vec_type>
[[nodiscard]] constexpr inline geometry::ray
reflect(const shape_type &shape, const geometry::ray &ray, float t) {
    auto base = ray.at(t);
    auto normal = shape.normal_at(base);
    auto out_dir = reflect(ray.dir, normal);

    return {base, out_dir};
}

struct material {
    color3 color = {1, 1, 1};
    color3 emission = {0, 0, 0};
    float roughness = 1.0;
    float metallic = 1.0;
    float ior = 1.5;
};

struct object_attrs {
    std::array<vec3, 3> normals;
};

struct Object {
    geometry::shape shape;
    geometry::object_attrs attrs;
    geometry::material material;

    [[nodiscard]] constexpr inline vec3 normal_at(const vec3 &pos) const {
        return shape.normal_at(pos);
    }

    [[nodiscard]] constexpr inline const color3 &emission() const {
        return material.emission;
    }

    [[nodiscard]] constexpr inline const color3 &color() const {
        return material.color;
    }

    [[nodiscard]] constexpr inline vec3 center() const {
        return shape.center();
    }

    [[nodiscard]] constexpr inline aabb bounding_box() const {
        return shape.bounding_box();
    }
};
} // namespace geometry

#endif // GEOMETRY_H
