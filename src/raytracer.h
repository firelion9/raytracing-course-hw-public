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
#include <variant>

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
    const geometry::Object *obj;
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
intersect_ray_obj(const geometry::ray &ray, const geometry::plane &obj) {
    intersection_vec res;

    float t =
        -geometry::dot(ray.start, obj.normal) / geometry::dot(ray.dir, obj.normal);

    push_t(t);

    return res;
}

static inline intersection_vec
intersect_ray_obj(const geometry::ray &ray,
                        const geometry::ellipsoid &obj) {
    intersection_vec res;

    float a = dot(ray.dir / obj.radius, ray.dir / obj.radius);
    float hb = dot(ray.start / obj.radius, ray.dir / obj.radius);
    float c = dot(ray.start / obj.radius, ray.start / obj.radius) - 1;
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
intersect_ray_obj(const geometry::ray &ray, const geometry::box &box) {
    intersection_vec res;

    auto intrs1 = (-box.half_size - ray.start) / ray.dir;
    auto intrs2 = (box.half_size - ray.start) / ray.dir;

    auto t_min = max_component(min(intrs1, intrs2));
    auto t_max = min_component(max(intrs1, intrs2));

    if (t_min <= t_max) {
        push_t(t_min);
        push_t(t_max);
    }

    return res;
}

static inline intersection_vec
intersect_unbiased(const geometry::ray &ray, const geometry::Object &obj) {
    intersection_vec res;

    return std::visit([&ray](const auto &shape) { return intersect_ray_obj(ray, shape); }, obj.shape);
}

static inline intersection_vec intersect(const geometry::ray &ray,
                                         const geometry::Object &obj) {
    geometry::quaternion rot = obj.rotation.conj();
    return intersect_unbiased(
        {(ray.start - obj.position) * rot, ray.dir * rot}, obj);
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
                                       const geometry::Object &obj,
                                       const intersection_vec &intersections) {
    if (intersections.size == 0)
        return;

    if (res.size == 0) {
        res.push({intersections[0], &obj});
    } else if (res[0].t > intersections[0]) {
        res[0] = {intersections[0], &obj};
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
                               : trace_res[0].obj->color;
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