#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "geometry.h"
#include "image.h"
#include "scene.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <ostream>
#include <stdexcept>
#include <vector>

template <size_t capacity, class T> struct small_vector {
    std::array<T, capacity> data;
    size_t size = 0;

    inline small_vector() = default;
    inline small_vector(const small_vector &) = default;

    inline void push(const T &e) { data[size++] = e; }

    inline T &operator[](size_t idx) { return data[idx]; }
    inline const T &operator[](size_t idx) const { return data[idx]; }

    inline auto begin() const { return data.begin(); }
    inline auto end() const { return data.begin() + size; }

    inline auto begin() { return data.begin(); }
    inline auto end() { return data.begin() + size; }
};

struct ray_intersection_info {
    float t = 0;
    const geometry::Shape *shape;
};

using intersection_vec = small_vector<2, float>;
using ray_trace_res = small_vector<1, ray_intersection_info>;

#define push_t(t)                                                              \
    do {                                                                       \
        if (t >= 0)                                                            \
            res.push(t);                                                       \
    } while (false)

static inline std::ostream &operator<<(std::ostream &out,
                                       const geometry::vec3 &v) {
    return out << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
}

static inline std::ostream &operator<<(std::ostream &out,
                                       const geometry::ray &v) {
    return out << v.start << "-" << v.dir << "->";
}

static inline intersection_vec
intersect_ray_plane(const geometry::ray &ray, const geometry::vec3 &normal) {
    intersection_vec res;

    float t =
        -geometry::dot(ray.start, normal) / geometry::dot(ray.dir, normal);

    push_t(t);

    return res;
}

static inline intersection_vec
intersect_ray_ellipsoid(const geometry::ray &ray,
                        const geometry::vec3 &radius) {
    intersection_vec res;

    float a = dot(ray.dir / radius, ray.dir / radius);
    float hb = dot(ray.start / radius, ray.dir / radius);
    float c = dot(ray.start / radius, ray.start / radius) - 1;
    float hd = sqrt(hb * hb - a * c);

    float t1 = (-hb - hd) / a;
    float t2 = (-hb + hd) / a;
    if (t1 > t2)
        std::swap(t1, t2);

    push_t(t1);
    push_t(t2);

    return res;
}

template <class vec_type>
static inline float max_component(const vec_type &vec) {
    return *std::max_element(vec.val.begin(), vec.val.end());
}

template <class vec_type>
static inline float min_component(const vec_type &vec) {
    return *std::min_element(vec.val.begin(), vec.val.end());
}

static inline intersection_vec
intersect_ray_box(const geometry::ray &ray, const geometry::vec3 &half_size) {
    intersection_vec res;

    auto intrs1 = (-half_size - ray.start) / ray.dir;
    auto intrs2 = (half_size - ray.start) / ray.dir;

    auto t_min = max_component(min(intrs1, intrs2));
    auto t_max = min_component(max(intrs1, intrs2));

    if (t_min <= t_max) {
        push_t(t_min);
        push_t(t_max);
    }

    return res;
}

static inline intersection_vec
intersect_unbiased(const geometry::ray &ray, const geometry::Shape &shape) {
    intersection_vec res;

    switch (shape.type) {
    case geometry::PLANE:
        return intersect_ray_plane(ray, shape.prop);

    case geometry::ELLIPSOID:
        return intersect_ray_ellipsoid(ray, shape.prop);

    case geometry::BOX:
        return intersect_ray_box(ray, shape.prop);

    default:
        throw std::runtime_error("unreachable code: unknown shape type");
    }
}

static inline intersection_vec intersect(const geometry::ray &ray,
                                         const geometry::Shape &shape) {
    geometry::quaternion rot = shape.rotation.conj();
    return intersect_unbiased(
        {(ray.start - shape.position) * rot, ray.dir * rot}, shape);
}

static inline geometry::ray gen_ray(const Camera &camera, int x, int y) {
    auto dir = norm((2 * (x + 0.5) / camera.width - 1) * tan(camera.fov_x / 2) *
                        camera.right -
                    (2 * (y + 0.5) / camera.height - 1) *
                        tan(camera.fov_y() / 2) * camera.up +
                    1 * camera.forward);

    return {camera.position, dir};
}

static inline void append_intersection(ray_trace_res &res,
                                       const geometry::Shape &shape,
                                       const intersection_vec &intersections) {
    if (intersections.size == 0)
        return;

    if (res.size == 0) {
        res.push({intersections[0], &shape});
    } else if (res[0].t > intersections[0]) {
        res[0] = {intersections[0], &shape};
    }
}

static inline void blend(geometry::color3 &screen_color,
                         geometry::color4 new_color) {
    screen_color =
        screen_color * (1 - new_color.a()) + new_color.rgb() * new_color.a();
}

static inline geometry::color4 cast_ray(const Scene &scene,
                                        const geometry::ray &ray) {
    ray_trace_res trace_res;

    for (const auto &shape : scene.objects) {
        append_intersection(trace_res, shape, intersect(ray, shape));
    }

    return trace_res.size == 0 ? geometry::color4(0, 0, 0, 0)
                               : trace_res[0].shape->color;
}

inline void run_raytracer(const Scene &scene, Image &image) {
    for (int y = 0; y < scene.camera.height; ++y) {
        for (int x = 0; x < scene.camera.width; ++x) {
            auto ray = gen_ray(scene.camera, x, y);

            blend(image.at(x, y), cast_ray(scene, ray));
        }
    }
}

#endif // RAYTRACER_H