#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "geometry.h"
#include "image.h"
#include "scene.h"
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <ostream>
#include <random>
#include <thread>
#include <type_traits>
#include <variant>
#include <vector>

// region config

constexpr bool USE_MULTITHREADING = true;
constexpr size_t SPAN_SIZE = 256;

constexpr float EPS = 1e-4;

constexpr struct recursive_ray_tracing_token_t {
} recursive_ray_tracing_token;
constexpr struct monte_carlo_token_t {
} monte_carlo_token;

constexpr auto ALGORITHM_TOKEN = monte_carlo_token;

// endregion

// region internal structures

struct RaytracerThreadContext {
    const Scene &scene;
    std::minstd_rand rand_gen;

    [[nodiscard]] inline geometry::vec2 uniform_offset2() {
        auto dist = std::uniform_real_distribution<float>(0.0f, 1.0f);
        return {
            dist(rand_gen),
            dist(rand_gen),
        };
    }

    [[nodiscard]] inline bool coin(float success_rate) {
        auto dist = std::uniform_real_distribution<float>(0.0f, 1.0f);
        return dist(rand_gen) <= success_rate;
    }

    [[nodiscard]] inline geometry::vec3 uniform_dir() {
        auto z_dist = std::uniform_real_distribution<float>(-1.0f, 1.0f);
        auto angle_dist = std::uniform_real_distribution<float>(
            0.0f, 2 * std::numbers::pi_v<float>);

        float z = z_dist(rand_gen);
        float co_z = std::sqrt(1 - z * z);
        float phi = angle_dist(rand_gen);

        return {co_z * std::cos(phi), co_z * std::sin(phi), z};
    }

    [[nodiscard]] inline geometry::vec3
    uniform_dir_semisphere(const geometry::vec3 &normal) {
        auto dir = uniform_dir();
        return geometry::dot(dir, normal) > 0 ? dir : -dir;
    }
};

struct ray_intersection_info {
    geometry::vec3 normal;
    float t = 0;
    const geometry::Object *obj;
    bool is_inside;
};

using intersection_res = std::optional<float>;
using ray_cast_res = std::optional<ray_intersection_info>;

// endregion

[[nodiscard]] constexpr static inline geometry::color3
trace_ray(RaytracerThreadContext &context, const geometry::ray &ray,
          unsigned max_depth);

// region utilities

template <class Num> [[nodiscard]] static constexpr inline Num pow2(Num x) {
    return x * x;
}

template <size_t p, class Num>
[[nodiscard]] static constexpr inline Num pow(Num x) {
    if constexpr (p == 0) {
        return 1;
    }
    if constexpr (p % 2 == 0) {
        return pow<p / 2>(x * x);
    } else {
        return x * pow<p / 2>(x * x);
    }
}

static inline std::ostream &operator<<(std::ostream &out,
                                       const geometry::vec3 &v) {
    return out << "(" << v.x() << ", " << v.y() << ", " << v.z() << ")";
}

static inline std::ostream &operator<<(std::ostream &out,
                                       const geometry::ray &v) {
    return out << v.start << "-" << v.dir << "->";
}

// endregion

// region intersections

template <class vec_type>
[[nodiscard]] constexpr static inline float max_component(const vec_type &vec) {
    return *std::max_element(vec.val.begin(), vec.val.end());
}

template <class vec_type>
[[nodiscard]] constexpr static inline float min_component(const vec_type &vec) {
    return *std::min_element(vec.val.begin(), vec.val.end());
}

#define push_t(t, min_dst)                                                     \
    do {                                                                       \
        if (t >= min_dst && (!res.has_value() || t < *res))                    \
            res = t;                                                           \
    } while (false)

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

// endregion

[[nodiscard]] constexpr static inline geometry::ray
gen_ray(const Camera &camera, int x, int y) {
    auto dir = norm((2 * (x + 0.5) / camera.width - 1) * tan(camera.fov_x / 2) *
                        camera.right -
                    (2 * (y + 0.5) / camera.height - 1) *
                        tan(camera.fov_y() / 2) * camera.up +
                    1 * camera.forward);

    return {camera.position, dir};
}

[[nodiscard]] static inline geometry::ray
gen_ray(RaytracerThreadContext &context, int x, int y) {
    auto &camera = context.scene.camera;
    auto offset = context.uniform_offset2();
    auto dir = norm((2 * (x + offset.x()) / camera.width - 1) *
                        tan(camera.fov_x / 2) * camera.right -
                    (2 * (y + offset.y()) / camera.height - 1) *
                        tan(camera.fov_y() / 2) * camera.up +
                    1 * camera.forward);

    return {camera.position, dir};
}

[[nodiscard]] static inline ray_cast_res
cast_ray(const RaytracerThreadContext &context, const geometry::ray &ray,
         float max_dst = std::numeric_limits<float>::infinity(),
         float min_dst = EPS) {
    ray_cast_res res;

    for (const auto &shape : context.scene.objects) {
        update_intersection(res, ray, shape, intersect(ray, shape, min_dst),
                            max_dst);
    }

    return res;
}

// region approx lighting

[[nodiscard]] static inline geometry::color3
light_at_from(const RaytracerThreadContext &context, const geometry::vec3 &pos,
              const geometry::vec3 &normal,
              const geometry::color3 &light_intensity,
              const geometry::point_light &light) {
    auto dir = light.position - pos;
    auto dst = dir.len();
    dir /= dst;
    auto cast_res = cast_ray(context, {pos, dir}, dst);
    if (!cast_res.has_value()) {
        return std::max(dot(normal, dir), 0.0f) * light_intensity /
               geometry::dot(light.attenuation, {1, dst, dst * dst});
    } else {
        return {0, 0, 0};
    }
}

[[nodiscard]] static inline geometry::color3
light_at_from(RaytracerThreadContext &context, const geometry::vec3 &pos,
              const geometry::vec3 &normal,
              const geometry::color3 &light_intensity,
              const geometry::directed_light &light) {
    auto cast_res = cast_ray(context, {pos, light.direction});
    if (!cast_res.has_value()) {
        return std::max(dot(normal, light.direction), 0.0f) * light_intensity;
    } else {
        return {0, 0, 0};
    }
}

[[nodiscard]] static inline geometry::color3
light_at_from(RaytracerThreadContext &context, const geometry::vec3 &pos,
              const geometry::vec3 &normal,
              const geometry::LightSource &light_source) {
    return std::visit(
        [&context, &pos, &normal, &light_source](const auto &light) {
            return light_at_from(context, pos, normal, light_source.intensity,
                                 light);
        },
        light_source.light);
}

[[nodiscard]]
static inline geometry::color3
shade(RaytracerThreadContext &context, const geometry::ray &ray,
      const ray_intersection_info &intersection_info, unsigned max_depth,
      geometry::diffuse, recursive_ray_tracing_token_t) {
    auto pos = ray.at(intersection_info.t);
    geometry::color3 res = context.scene.ambient_light;

    for (const auto &light : context.scene.lights) {
        res += light_at_from(context, pos, intersection_info.normal, light);
    }

    return res * intersection_info.obj->color;
}

[[nodiscard]] constexpr static inline geometry::color3
shade(RaytracerThreadContext &context, const geometry::ray &ray,
      const ray_intersection_info &intersection_info, unsigned max_depth,
      geometry::metallic, recursive_ray_tracing_token_t) {
    auto pos = ray.at(intersection_info.t);
    auto res = trace_ray(
        context, {pos, geometry::reflect(intersection_info.normal, ray.dir)},
        max_depth);

    return res * intersection_info.obj->color;
}

[[nodiscard]] constexpr static inline geometry::color3
shade(RaytracerThreadContext &context, const geometry::ray &ray,
      const ray_intersection_info &intersection_info, unsigned max_depth,
      geometry::dielectric material, recursive_ray_tracing_token_t) {
    auto pos = ray.at(intersection_info.t);

    float r0 = pow2((1 - material.ior) / (1 + material.ior));
    float r =
        r0 + (1 - r0) * pow<5>(1 - dot(intersection_info.normal, -ray.dir));

    geometry::color3 res = trace_ray(
        context, {pos, geometry::reflect(intersection_info.normal, ray.dir)},
        max_depth);

    float eta = intersection_info.is_inside ? material.ior : 1 / material.ior;
    auto sin_theta2_2 =
        eta * eta * (1 - pow2(dot(intersection_info.normal, -ray.dir)));
    if (sin_theta2_2 <= 1) {
        auto cos_theta2 = std::sqrt(1 - sin_theta2_2);
        auto dir1 = geometry::norm(
            eta * ray.dir +
            (eta * dot(intersection_info.normal, -ray.dir) - cos_theta2) *
                intersection_info.normal);

        res = r * res + (1 - r) * trace_ray(context, {pos, dir1}, max_depth) *
                            (intersection_info.is_inside
                                 ? geometry::color3{1, 1, 1}
                                 : intersection_info.obj->color);
    }

    return res;
}

// endregion

// region Monte-Carlo lighting

[[nodiscard]]
static inline geometry::color3
shade(RaytracerThreadContext &context, const geometry::ray &ray,
      const ray_intersection_info &intersection_info, unsigned max_depth,
      const geometry::diffuse &material, monte_carlo_token_t) {
    auto pos = ray.at(intersection_info.t);
    auto dir = context.uniform_dir_semisphere(intersection_info.normal);

    return material.emission + 2 * intersection_info.obj->color *
                                   trace_ray(context, {pos, dir}, max_depth) *
                                   geometry::dot(dir, intersection_info.normal);
}

[[nodiscard]] constexpr static inline geometry::color3
shade(RaytracerThreadContext &context, const geometry::ray &ray,
      const ray_intersection_info &intersection_info, unsigned max_depth,
      const geometry::metallic &material, monte_carlo_token_t) {
    auto pos = ray.at(intersection_info.t);
    auto res = trace_ray(
        context, {pos, geometry::reflect(intersection_info.normal, ray.dir)},
        max_depth);

    return material.emission + res * intersection_info.obj->color;
}

[[nodiscard]] constexpr static inline geometry::color3
shade(RaytracerThreadContext &context, const geometry::ray &ray,
      const ray_intersection_info &intersection_info, unsigned max_depth,
      const geometry::dielectric &material, monte_carlo_token_t) {
    auto pos = ray.at(intersection_info.t);

    float r0 = pow2((1 - material.ior) / (1 + material.ior));
    float r =
        r0 + (1 - r0) * pow<5>(1 - dot(intersection_info.normal, -ray.dir));

    float eta = intersection_info.is_inside ? material.ior : 1 / material.ior;
    auto sin_theta2_2 =
        eta * eta * (1 - pow2(dot(intersection_info.normal, -ray.dir)));

    if (sin_theta2_2 > 1 || context.coin(r)) {
        return material.emission +
               trace_ray(
                   context,
                   {pos, geometry::reflect(intersection_info.normal, ray.dir)},
                   max_depth);
    } else {
        auto cos_theta2 = std::sqrt(1 - sin_theta2_2);
        auto dir1 = geometry::norm(
            eta * ray.dir +
            (eta * dot(intersection_info.normal, -ray.dir) - cos_theta2) *
                intersection_info.normal);

        return material.emission + trace_ray(context, {pos, dir1}, max_depth) *
                                       (intersection_info.is_inside
                                            ? geometry::color3{1, 1, 1}
                                            : intersection_info.obj->color);
    }
}

// endregion

[[nodiscard]] constexpr static inline geometry::color3
shade(RaytracerThreadContext &context, const geometry::ray &ray,
      const ray_intersection_info &intersection_info, unsigned max_depth) {
    return std::visit(
        [&context, &ray, &intersection_info, &max_depth](auto mat) {
            return shade(context, ray, intersection_info, max_depth, mat,
                         ALGORITHM_TOKEN);
        },
        intersection_info.obj->material);
}

[[nodiscard]] constexpr static inline geometry::color3
trace_ray(RaytracerThreadContext &context, const geometry::ray &ray,
          unsigned max_depth) {
    if (max_depth == 0) {
        return {0, 0, 0};
    }

    ray_cast_res trace_res = cast_ray(context, ray);

    return trace_res.has_value()
               ? shade(context, ray, trace_res.value(), max_depth - 1)
               : context.scene.bg_color;
}

[[nodiscard]] static inline geometry::color3
render_pixel(RaytracerThreadContext &context, int x, int y) {
    if constexpr (std::is_same_v<std::remove_const_t<decltype(ALGORITHM_TOKEN)>,
                                 monte_carlo_token_t>) {
        geometry::color3 res = {0, 0, 0};
        for (int s = 0; s < context.scene.samples; ++s) {
            auto ray = gen_ray(context, x, y);
            res += trace_ray(context, ray, context.scene.ray_depth);
        }
        return res / context.scene.samples;
    } else {
        auto ray = gen_ray(context.scene.camera, x, y);
        return trace_ray(context, ray, context.scene.ray_depth);
    }
}

inline void run_raytracer(const Scene &scene, Image &image) {
    if (scene.ray_depth == 0)
        return;

    if constexpr (USE_MULTITHREADING) {
        auto worker_count = std::thread::hardware_concurrency();
        std::vector<std::thread> workers;
        workers.reserve(worker_count);
        std::atomic_int next_span(0);
        int span_count = (image.data.size() + SPAN_SIZE - 1) / SPAN_SIZE;
        auto span = (image.data.size() + worker_count - 1) / worker_count;
        for (int i = 0; i < worker_count; ++i) {
            workers.push_back(std::thread([i, &scene, &image, &next_span,
                                           &span_count]() {
                int span = -1;
                while ((span = next_span.fetch_add(1)) < span_count) {
                    RaytracerThreadContext context{scene,
                                                   std::minstd_rand(span)};
                    auto begin = SPAN_SIZE * span,
                         end = std::min(begin + SPAN_SIZE, image.data.size());
                    for (int p_idx = begin, x = p_idx % image.width,
                             y = p_idx / image.width;
                         p_idx < end; ++p_idx, ++x) {
                        if (x == image.width) {
                            x = 0;
                            y += 1;
                        }
                        image.set_pixel(p_idx, render_pixel(context, x, y));
                    }
                }
            }));
        }

        for (auto &th : workers)
            th.join();
    } else {
        RaytracerThreadContext context{scene, std::minstd_rand(42)};
        for (int y = 0; y < scene.camera.height; ++y) {
            for (int x = 0; x < scene.camera.width; ++x) {
                image.set_pixel(x, y, render_pixel(context, x, y));
            }
        }
    }
}

#endif // RAYTRACER_H