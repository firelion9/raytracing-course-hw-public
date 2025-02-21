#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "geometry.h"
#include "image.h"
#include "scene.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <optional>
#include <ostream>
#include <thread>
#include <variant>
#include <vector>

constexpr float EPS = 1e-4;

struct ray_intersection_info {
    geometry::vec3 normal;
    float t = 0;
    const geometry::Object *obj;
    bool is_inside;
};

using intersection_res = std::optional<float>;
using ray_cast_res = std::optional<ray_intersection_info>;

[[nodiscard]] constexpr static inline geometry::color3
trace_ray(const Scene &scene, const geometry::ray &ray, unsigned max_depth);

template <class Num> [[nodiscard]] constexpr inline Num pow2(Num x) {
    return x * x;
}

template <size_t p, class Num> [[nodiscard]] constexpr inline Num pow(Num x) {
    if constexpr (p == 0) {
        return 1;
    }
    if constexpr (p % 2 == 0) {
        return pow<p / 2>(x * x);
    } else {
        return x * pow<p / 2>(x * x);
    }
}

#define push_t(t, min_dst)                                                     \
    do {                                                                       \
        if (t >= min_dst && (!res.has_value() || t < *res))                    \
            res = t;                                                           \
    } while (false)

static inline std::ostream &operator<<(std::ostream &out,
                                       const geometry::vec3 &v) {
    return out << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
}

static inline std::ostream &operator<<(std::ostream &out,
                                       const geometry::ray &v) {
    return out << v.start << "-" << v.dir << "->";
}

[[nodiscard]] constexpr static inline intersection_res
intersect_ray_obj(const geometry::ray &ray, const geometry::plane &obj,
                  float min_dst) {
    intersection_res res;

    float t = -geometry::dot(ray.start, obj.normal) /
              geometry::dot(ray.dir, obj.normal);

    push_t(t, min_dst);

    return res;
}

[[nodiscard]] constexpr static inline intersection_res
intersect_ray_obj(const geometry::ray &ray, const geometry::ellipsoid &obj,
                  float min_dst) {
    intersection_res res;

    float a = dot(ray.dir / obj.radius, ray.dir / obj.radius);
    float hb = dot(ray.start / obj.radius, ray.dir / obj.radius);
    float c = dot(ray.start / obj.radius, ray.start / obj.radius) - 1;
    float hd = sqrt(hb * hb - a * c);

    float t1 = (-hb - hd) / a;
    float t2 = (-hb + hd) / a;

    push_t(t1, min_dst);
    push_t(t2, min_dst);

    return res;
}

template <class vec_type>
[[nodiscard]] constexpr static inline float max_component(const vec_type &vec) {
    return *std::max_element(vec.val.begin(), vec.val.end());
}

template <class vec_type>
[[nodiscard]] constexpr static inline float min_component(const vec_type &vec) {
    return *std::min_element(vec.val.begin(), vec.val.end());
}

[[nodiscard]] constexpr static inline intersection_res
intersect_ray_obj(const geometry::ray &ray, const geometry::box &box,
                  float min_dst) {
    intersection_res res;

    auto intrs1 = (-box.half_size - ray.start) / ray.dir;
    auto intrs2 = (box.half_size - ray.start) / ray.dir;

    auto t_min = max_component(min(intrs1, intrs2));
    auto t_max = min_component(max(intrs1, intrs2));

    if (t_min <= t_max) {
        push_t(t_min, min_dst);
        push_t(t_max, min_dst);
    }

    return res;
}

[[nodiscard]] constexpr static inline intersection_res
intersect_unbiased(const geometry::ray &ray, const geometry::Object &obj,
                   float min_dst) {
    return std::visit(
        [&ray, min_dst](const auto &shape) {
            return intersect_ray_obj(ray, shape, min_dst);
        },
        obj.shape);
}

[[nodiscard]] constexpr static inline intersection_res
intersect(const geometry::ray &ray, const geometry::Object &obj,
          float min_dst) {
    geometry::quaternion rot = obj.rotation.conj();
    return intersect_unbiased({(ray.start - obj.position) * rot, ray.dir * rot},
                              obj, min_dst);
}

[[nodiscard]] constexpr static inline geometry::ray
gen_ray(const Camera &camera, int x, int y) {
    auto dir = norm((2 * (x + 0.5) / camera.width - 1) * tan(camera.fov_x / 2) *
                        camera.right -
                    (2 * (y + 0.5) / camera.height - 1) *
                        tan(camera.fov_y() / 2) * camera.up +
                    1 * camera.forward);

    return {camera.position, dir};
}

static inline void update_intersection(ray_cast_res &res,
                                       const geometry::ray &ray,
                                       const geometry::Object &obj,
                                       const intersection_res &intersection,
                                       float max_dst) {
    if (!intersection.has_value() || *intersection > max_dst)
        return;

    if (!res.has_value() || res->t > *intersection) {
        auto pos = ray.at(*intersection);
        auto normal = obj.normal_at(pos);
        auto is_inside = geometry::dot(normal, ray.dir) > 0;
        res = {is_inside ? -normal : normal, *intersection, &obj, is_inside};
    }
}

[[nodiscard]] static inline ray_cast_res
cast_ray(const Scene &scene, const geometry::ray &ray,
         float max_dst = std::numeric_limits<float>::infinity(),
         float min_dst = EPS) {
    ray_cast_res res;

    for (const auto &shape : scene.objects) {
        update_intersection(res, ray, shape, intersect(ray, shape, min_dst),
                            max_dst);
    }

    return res;
}

[[nodiscard]] static inline geometry::color3
light_at_from(const Scene &scene, const geometry::vec3 &pos,
              const geometry::vec3 &normal,
              const geometry::color3 &light_intensity,
              const geometry::point_light &light) {
    auto dir = light.position - pos;
    auto dst = dir.len();
    dir /= dst;
    auto cast_res = cast_ray(scene, {pos, dir}, dst);
    if (!cast_res.has_value()) {
        return std::max(dot(normal, dir), 0.0f) * light_intensity /
               geometry::dot(light.attenuation, {1, dst, dst * dst});
    } else {
        return {0, 0, 0};
    }
}

[[nodiscard]] static inline geometry::color3
light_at_from(const Scene &scene, const geometry::vec3 &pos,
              const geometry::vec3 &normal,
              const geometry::color3 &light_intensity,
              const geometry::directed_light &light) {
    auto cast_res = cast_ray(scene, {pos, light.direction});
    if (!cast_res.has_value()) {
        return std::max(dot(normal, light.direction), 0.0f) * light_intensity;
    } else {
        return {0, 0, 0};
    }
}

[[nodiscard]] static inline geometry::color3
light_at_from(const Scene &scene, const geometry::vec3 &pos,
              const geometry::vec3 &normal,
              const geometry::LightSource &light_source) {
    return std::visit(
        [&scene, &pos, &normal, &light_source](const auto &light) {
            return light_at_from(scene, pos, normal, light_source.intensity,
                                 light);
        },
        light_source.light);
}

[[nodiscard]]
static inline geometry::color3
shade(const Scene &scene, const geometry::ray &ray,
      const ray_intersection_info &intersection_info, unsigned max_depth,
      geometry::diffuse) {
    auto pos = ray.at(intersection_info.t);
    geometry::color3 res = scene.ambient_light;

    for (const auto &light : scene.lights) {
        res += light_at_from(scene, pos, intersection_info.normal, light);
    }

    return res * intersection_info.obj->color;
}

[[nodiscard]] constexpr static inline geometry::color3
shade(const Scene &scene, const geometry::ray &ray,
      const ray_intersection_info &intersection_info, unsigned max_depth,
      geometry::metallic) {
    auto pos = ray.at(intersection_info.t);
    auto res = trace_ray(
        scene, {pos, geometry::reflect(intersection_info.normal, ray.dir)},
        max_depth);

    return res * intersection_info.obj->color;
}

[[nodiscard]] constexpr static inline geometry::color3
shade(const Scene &scene, const geometry::ray &ray,
      const ray_intersection_info &intersection_info, unsigned max_depth,
      geometry::dielectric material) {
    auto pos = ray.at(intersection_info.t);

    float r0 = pow2((1 - material.ior) / (1 + material.ior));
    float r =
        r0 + (1 - r0) * pow<5>(1 - dot(intersection_info.normal, -ray.dir));

    geometry::color3 res =
        trace_ray(scene,
                  {pos, geometry::reflect(intersection_info.normal, ray.dir)},
                  max_depth) *
        r;

    float eta = intersection_info.is_inside ? material.ior : 1 / material.ior;
    auto sin_theta2_2 =
        eta * eta * (1 - pow2(dot(intersection_info.normal, -ray.dir)));
    if (sin_theta2_2 <= 1) {
        auto cos_theta2 = std::sqrt(1 - sin_theta2_2);
        auto dir1 = geometry::norm(
            eta * ray.dir +
            (eta * dot(intersection_info.normal, -ray.dir) - cos_theta2) *
                intersection_info.normal);

        res += (1 - r) * trace_ray(scene, {pos, dir1}, max_depth) *
               (intersection_info.is_inside ? geometry::color3{1, 1, 1}
                                            : intersection_info.obj->color);
    }

    return res;
}

[[nodiscard]] constexpr static inline geometry::color3
shade(const Scene &scene, const geometry::ray &ray,
      const ray_intersection_info &intersection_info, unsigned max_depth) {
    return std::visit(
        [&scene, &ray, &intersection_info, &max_depth](auto mat) {
            return shade(scene, ray, intersection_info, max_depth, mat);
        },
        intersection_info.obj->material);
}

[[nodiscard]] constexpr static inline geometry::color3
trace_ray(const Scene &scene, const geometry::ray &ray, unsigned max_depth) {
    if (max_depth == 0) {
        return scene.bg_color;
    }

    ray_cast_res trace_res = cast_ray(scene, ray);

    return trace_res.has_value()
               ? shade(scene, ray, trace_res.value(), max_depth - 1)
               : scene.bg_color;
}

inline void run_raytracer(const Scene &scene, Image &image) {
    if (scene.ray_depth == 0)
        return;

    // auto worker_count = std::thread::hardware_concurrency();
    // std::vector<std::thread> workers;
    // workers.reserve(worker_count);
    // auto span = (image.data.size() + worker_count - 1) / worker_count;
    // for (int i = 0; i < worker_count; ++i) {
    //     workers.push_back(std::thread([&scene, &image, i, span]() {
    //         auto begin = span * i,
    //              end = std::min(span * (i + 1), image.data.size());
    //         for (int j = begin; j < end; ++j) {
    //             int x = j % image.width;
    //             int y = j / image.width;
    //         auto ray = gen_ray(scene.camera, x, y);

    //             image.at(x, y) = trace_ray(scene, ray, scene.ray_depth);
    //         }
    //     }));
    // }

    // for (auto &th : workers) th.join();

    for (int y = 0; y < scene.camera.height; ++y) {
        for (int x = 0; x < scene.camera.width; ++x) {
            auto ray = gen_ray(scene.camera, x, y);

            image.at(x, y) = trace_ray(scene, ray, scene.ray_depth);
        }
    }
}

#endif // RAYTRACER_H