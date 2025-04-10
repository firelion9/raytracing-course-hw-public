#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "geometry.h"
#include "image.h"
#include "scene.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cmath>
#include <iterator>
#include <limits>
#include <numbers>
#include <numeric>
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
template <size_t capacity, class T> struct small_vector {
    std::array<T, capacity> data;
    size_t size = 0;

    [[nodiscard]] constexpr inline small_vector() = default;
    [[nodiscard]] constexpr inline small_vector(const small_vector &) = default;

    constexpr inline void push(const T &e) { data[size++] = e; }

    [[nodiscard]] constexpr inline T &operator[](size_t idx) {
        return data[idx];
    }
    [[nodiscard]] constexpr inline const T &operator[](size_t idx) const {
        return data[idx];
    }

    [[nodiscard]] constexpr inline auto begin() const { return data.begin(); }
    [[nodiscard]] constexpr inline auto end() const {
        return data.begin() + size;
    }

    [[nodiscard]] constexpr inline auto begin() { return data.begin(); }
    [[nodiscard]] constexpr inline auto end() { return data.begin() + size; }

    [[nodiscard]] constexpr inline auto empty() const { return size == 0; }

    [[nodiscard]] constexpr inline auto min() const {
        auto res = data[0];
        for (int i = 1; i < size; ++i)
            res = std::min(res, data[i]);
        return res;
    }
};

struct ray_intersection_info {
    geometry::vec3 normal;
    float t = 0;
    const geometry::Object *obj;
    bool is_inside;
};

using intersection_res = small_vector<2, float>;
using ray_cast_res = std::optional<ray_intersection_info>;

[[nodiscard]] constexpr static inline intersection_res
intersect_ray_obj(const geometry::ray &ray, const geometry::box &box,
                  float min_dst);
[[nodiscard]] constexpr static inline intersection_res
intersect_ray_obj(const geometry::ray &ray,
                  const geometry::ellipsoid &ellipsoid, float min_dst);
[[nodiscard]] constexpr static inline intersection_res
intersect_ray_obj(const geometry::ray &ray,
                  const geometry::triangle &triangle, float min_dst);

struct cosine_dist;
struct light_dist;
struct mix_dist;
using dist_t = std::variant<cosine_dist, light_dist, mix_dist>;

[[nodiscard]] constexpr static inline float light_surface_projection_multiplier(
    const geometry::vec3 &center, const geometry::vec3 &y,
    const geometry::vec3 &normal_y,
    const geometry::vec3 &dir /* = norm(y - center) */) {
    return (center - y).len2() / std::abs(geometry::dot(dir, normal_y));
}

struct sphere_uniform_dist {
    template <class Rng>
    [[nodiscard]] inline geometry::vec3
    sample(Rng &rng, const geometry::vec3 &x,
           const geometry::vec3 &normal) const {
        auto z_dist = std::uniform_real_distribution<float>(-1.0f, 1.0f);
        auto angle_dist = std::uniform_real_distribution<float>(
            0.0f, 2 * std::numbers::pi_v<float>);

        float z = z_dist(rng);
        float co_z = std::sqrt(1 - z * z);
        float phi = angle_dist(rng);

        return {co_z * std::cos(phi), co_z * std::sin(phi), z};
    }

    [[nodiscard]] inline float pdf(const geometry::vec3 &x,
                                   const geometry::vec3 &normal,
                                   const geometry::vec3 &dir) const {
        return 1.0f / 4 / std::numbers::inv_pi_v<float>;
    }
};

struct cosine_dist {
    template <class Rng>
    [[nodiscard]] constexpr inline geometry::vec3
    sample(Rng &rng, const geometry::vec3 &x,
           const geometry::vec3 &normal) const {
        auto dir = normal + sphere_uniform_dist().sample(rng, x, normal);
        return geometry::norm(dir);
    }

    [[nodiscard]] inline float pdf(const geometry::vec3 &x,
                                   const geometry::vec3 &normal,
                                   const geometry::vec3 &dir) const {
        return std::max(geometry::dot(normal, dir) / std::numbers::pi_v<float>,
                        0.0f);
    }
};

struct box_dist {
    geometry::box box;
    geometry::vec2 weights;
    float area_inv;

    [[nodiscard]] constexpr inline box_dist(const geometry::box &box)
        : box(box) {
        geometry::vec3 weights{box.half_size.y() * box.half_size.z(),
                               box.half_size.z() * box.half_size.x(),
                               box.half_size.x() * box.half_size.y()};
        weights.y() += weights.x();
        weights.z() += weights.y();
        this->weights = weights.xy() / weights.z();
        area_inv = 1 / (8 * weights.z());
    }

    template <class Rng>
    [[nodiscard]] inline geometry::vec3
    sample(Rng &rng, const geometry::vec3 &x,
           const geometry::vec3 &normal) const {
        float face_cont = std::uniform_real_distribution<float>()(rng);
        int face_idx = face_cont >= weights.y()   ? 2
                       : face_cont >= weights.x() ? 1
                                                  : 0;
        const std::array<geometry::vec3, 3> &face_transform =
            face_index_to_transform[face_idx];

        auto off_dist = std::uniform_real_distribution<float>(-1, 1);
        auto p = box.half_size *
                 (face_transform[0] * (off_dist(rng) >= 0.0 ? 1 : -1) +
                  face_transform[1] * off_dist(rng) +
                  face_transform[2] * off_dist(rng));
        return geometry::norm(p - x);
    }

    [[nodiscard]] inline float pdf(const geometry::vec3 &x,
                                   const geometry::vec3 &normal,
                                   const geometry::vec3 &dir) const {
        geometry::ray ray{x, dir};
        auto intersections = intersect_ray_obj(ray, box, EPS);

        float res = 0;
        for (auto t : intersections) {
            auto y = ray.at(t);
            res += light_surface_projection_multiplier(x, y, box.normal_at(y),
                                                       dir);
        }
        return res * area_inv;
    }

    static constexpr std::array<std::array<geometry::vec3, 3>, 3>
        face_index_to_transform{
            std::array<geometry::vec3, 3>{
                geometry::vec3{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
            std::array<geometry::vec3, 3>{
                geometry::vec3{0, 1, 0}, {0, 0, 1}, {1, 0, 0}},
            std::array<geometry::vec3, 3>{
                geometry::vec3{0, 0, 1}, {1, 0, 0}, {0, 1, 0}},
        };
};

struct ellipsoid_dist {
    geometry::ellipsoid ellipsoid;

    template <class Rng>
    [[nodiscard]] inline geometry::vec3
    sample(Rng &rng, const geometry::vec3 &x,
           const geometry::vec3 &normal) const {
        auto p =
            ellipsoid.radius * sphere_uniform_dist().sample(rng, x, normal);
        return geometry::norm(p - x);
    }

    [[nodiscard]] inline float pdf(const geometry::vec3 &x,
                                   const geometry::vec3 &normal,
                                   const geometry::vec3 &dir) const {
        geometry::ray ray{x, dir};
        auto intersections = intersect_ray_obj(ray, ellipsoid, EPS);
        float res = 0;
        for (auto t : intersections) {
            auto y = ray.at(t);
            auto n = y / ellipsoid.radius;
            res += light_surface_projection_multiplier(
                       x, y, ellipsoid.normal_at(y), dir) /
                   (n * ellipsoid.radius.yzx() * ellipsoid.radius.zxy()).len();
        }
        return res / (4 * std::numbers::pi_v<float>);
    }
};

struct triangle_dist {
    geometry::triangle triangle;

    template <class Rng>
    [[nodiscard]] inline geometry::vec3
    sample(Rng &rng, const geometry::vec3 &x,
           const geometry::vec3 &normal) const {
        auto dist = std::uniform_real_distribution<float>(0, 1);
        float u = dist(rng);
        float v = dist(rng);
        if (u + v > 1) {
            u = 1 - u;
            v = 1 - v;
        }

        auto p = triangle.a() + triangle.v() * v + triangle.u() * u;
        return geometry::norm(p - x);
    }

    [[nodiscard]] inline float pdf(const geometry::vec3 &x,
                                   const geometry::vec3 &normal,
                                   const geometry::vec3 &dir) const {
        geometry::ray ray{x, dir};
        auto intersections = intersect_ray_obj(ray, triangle, EPS);
        float res = 0;
        for (auto t : intersections) {
            auto y = ray.at(t);
            res += light_surface_projection_multiplier(x, y, triangle.normal_at(y), dir);
        }
        return res / triangle.square();
    }
};

struct light_dist {
    geometry::vec3 pos;
    geometry::quaternion rotation;
    std::variant<box_dist, ellipsoid_dist, triangle_dist> dist;

    template <class Rng>
    [[nodiscard]] inline geometry::vec3
    sample(Rng &rng, const geometry::vec3 &x,
           const geometry::vec3 &normal) const {
        return std::visit(
            [&pos = this->pos, &rotation = this->rotation, &rng, &x,
             &normal](const auto &dist) {
                return dist.sample(rng, (x - pos) * rotation.conj(),
                                   normal * rotation.conj()) *
                       rotation;
            },
            dist);
    }

    [[nodiscard]] inline float pdf(const geometry::vec3 &x,
                                   const geometry::vec3 &normal,
                                   const geometry::vec3 &dir) const {
        return std::visit(
            [&pos = this->pos, &rotation = this->rotation, &x, &normal,
             &dir](const auto &dist) {
                return dist.pdf((x - pos) * rotation.conj(),
                                normal * rotation.conj(),
                                dir * rotation.conj());
            },
            dist);
    }
};

struct mix_dist {
    std::vector<dist_t> dists;

    template <class Rng>
    [[nodiscard]] inline geometry::vec3
    sample(Rng &rng, const geometry::vec3 &x,
           const geometry::vec3 &normal) const {
        auto dist =
            dists[std::uniform_int_distribution<>(0, dists.size() - 1)(rng)];

        return std::visit(
            [&rng, &x, &normal](const auto &dist) {
                return dist.sample(rng, x, normal);
            },
            dist);
    }

    [[nodiscard]] inline float pdf(const geometry::vec3 &x,
                                   const geometry::vec3 &normal,
                                   const geometry::vec3 &dir) const {
        float res = 0;
        for (auto &dist : dists) {
            res += std::visit(
                [&x, &normal, &dir](const auto &dist) {
                    return dist.pdf(x, normal, dir);
                },
                dist);
        }
        return res / dists.size();
    }
};

template <class Rng> struct dir_generator {
    Rng &rng;
    dist_t dist;

    [[nodiscard]] inline geometry::vec3 sample(const geometry::vec3 &x,
                                               const geometry::vec3 &normal) {
        return std::visit(
            [&rng = this->rng, &x, &normal](const auto &dist) {
                return dist.sample(rng, x, normal);
            },
            dist);
    }

    [[nodiscard]] inline float pdf(const geometry::vec3 &x,
                                   const geometry::vec3 &normal,
                                   const geometry::vec3 &dir) const {
        return std::visit(
            [&x, &normal, &dir](const auto &dist) {
                return dist.pdf(x, normal, dir);
            },
            dist);
    }
};

struct RaytracerThreadContext {
    const Scene &scene;
    std::minstd_rand rand_gen;
    dir_generator<std::minstd_rand> dir_gen;

    RaytracerThreadContext(const Scene &scene, const dist_t &dir_gen,
                           int rand_seed)
        : scene(scene), rand_gen(rand_seed), dir_gen({rand_gen, dir_gen}) {}

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
        if (t >= min_dst)                                                      \
            res.push(t);                                                       \
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
intersect_ray_obj(const geometry::ray &ray, const geometry::triangle &triangle,
                  float min_dst) {
    intersection_res res;

    auto av = triangle.v();
    auto au = triangle.u();
    auto at = -ray.dir;
    auto y = ray.start - triangle.a();

    auto xs = geometry::vec3(geometry::det(y, au, at), geometry::det(av, y, at),
                             geometry::det(av, au, y)) /
              geometry::det(av, au, at);

    if (xs.x() >= 0 && xs.y() >= 0 && xs.x() + xs.y() <= 1 && xs.z() >= 0) {
        push_t(xs.z(), min_dst);
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
    if (intersection.empty())
        return;
    auto t = intersection.min();
    if (t > max_dst)
        return;

    if (!res.has_value() || res->t > t) {
        auto pos = ray.at(t);
        auto normal = obj.normal_at(pos);
        auto is_inside = geometry::dot(normal, ray.dir) > 0;
        res = {is_inside ? -normal : normal, t, &obj, is_inside};
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
    auto dir = context.dir_gen.sample(pos, intersection_info.normal);
    auto p = context.dir_gen.pdf(pos, intersection_info.normal, dir);
    if (p == 0.0) {
        return material.emission;
    }

    return material.emission +
           intersection_info.obj->color / std::numbers::pi_v<float> *
               trace_ray(context, {pos, dir}, max_depth) *
               geometry::dot(dir, intersection_info.normal) / p;
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

    dist_t diffuse_dist = cosine_dist();
    std::vector<dist_t> light_sources;
    for (auto &obj : scene.objects) {
        if (obj.emission() == geometry::color3{0, 0, 0})
            continue;

        std::visit(
            [&pos = obj.position, &rotation = obj.rotation,
             &light_sources](auto shape) {
                if constexpr (std::is_same_v<decltype(shape),
                                             geometry::ellipsoid>) {
                    light_sources.push_back(
                        light_dist{pos, rotation, ellipsoid_dist{shape}});
                } else if constexpr (std::is_same_v<decltype(shape),
                                                    geometry::box>) {
                    light_sources.push_back(
                        light_dist{pos, rotation, box_dist(shape)});
                } else if constexpr (std::is_same_v<decltype(shape),
                                                    geometry::triangle>) {
                    light_sources.push_back(
                        light_dist{pos, rotation, triangle_dist(shape)});
                }
            },
            obj.shape);
    }
    if (!light_sources.empty()) {
        diffuse_dist = mix_dist{{diffuse_dist, mix_dist{light_sources}}};
    }

    if constexpr (USE_MULTITHREADING) {
        auto worker_count = std::thread::hardware_concurrency();
        std::vector<std::thread> workers;
        workers.reserve(worker_count);
        std::atomic_int next_span(0);
        int span_count = (image.data.size() + SPAN_SIZE - 1) / SPAN_SIZE;
        auto span = (image.data.size() + worker_count - 1) / worker_count;
        for (int i = 0; i < worker_count; ++i) {
            workers.push_back(std::thread([i, &scene, &image, &next_span,
                                           &span_count, &diffuse_dist]() {
                int span = -1;
                while ((span = next_span.fetch_add(1)) < span_count) {
                    RaytracerThreadContext context(scene, diffuse_dist, span);
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
        RaytracerThreadContext context(scene, diffuse_dist, 42);
        for (int y = 0; y < scene.camera.height; ++y) {
            for (int x = 0; x < scene.camera.width; ++x) {
                image.set_pixel(x, y, render_pixel(context, x, y));
            }
        }
    }
}

#endif // RAYTRACER_H