#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "bvh.h"
#include "geometry.h"
#include "image.h"
#include "scene.h"
#include <algorithm>
#include <array>
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

struct cosine_dist;
struct light_dist;
struct mix_dist;
struct bvh_mix_dist;
using dist_t = std::variant<cosine_dist, light_dist, mix_dist, bvh_mix_dist>;

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

    [[nodiscard]] constexpr inline float pdf_at(const geometry::vec3 &x,
                                                const geometry::vec3 &normal,
                                                const geometry::vec3 &y) const {
        return light_surface_projection_multiplier(x, y, box.normal_at(y),
                                                   geometry::norm(y - x)) *
               area_inv;
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

    [[nodiscard]] constexpr inline float pdf_at(const geometry::vec3 &x,
                                                const geometry::vec3 &normal,
                                                const geometry::vec3 &y) const {
        auto n = y / ellipsoid.radius;
        return light_surface_projection_multiplier(x, y, ellipsoid.normal_at(y),
                                                   geometry::norm(y - x)) /
               (n * ellipsoid.radius.yzx() * ellipsoid.radius.zxy()).len() /
               (4 * std::numbers::pi_v<float>);
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

    [[nodiscard]] constexpr inline float pdf_at(const geometry::vec3 &x,
                                                const geometry::vec3 &normal,
                                                const geometry::vec3 &y) const {
        return light_surface_projection_multiplier(x, y, triangle.normal_at(y),
                                                   geometry::norm(y - x)) /
               triangle.square();
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
    [[nodiscard]] constexpr inline float pdf_at(const geometry::vec3 &x,
                                                const geometry::vec3 &normal,
                                                const geometry::vec3 &y) const {
        return std::visit(
            [&pos = this->pos, &rotation = this->rotation, &x, &normal,
             &y](const auto &dist) {
                return dist.pdf_at((x - pos) * rotation.conj(),
                                   normal * rotation.conj(),
                                   (y - pos) * rotation.conj());
            },
            dist);
    }
};

[[nodiscard]] static constexpr inline light_dist
make_light_dist(const geometry::Object &obj) {
    return std::visit(
        [&pos = obj.position, &rotation = obj.rotation](const auto &shape) {
            if constexpr (std::is_same_v<std::remove_cvref_t<decltype(shape)>,
                                         geometry::ellipsoid>) {
                return light_dist{pos, rotation, ellipsoid_dist{shape}};
            } else if constexpr (std::is_same_v<
                                     std::remove_cvref_t<decltype(shape)>,
                                     geometry::box>) {
                return light_dist{pos, rotation, box_dist(shape)};
            } else if constexpr (std::is_same_v<
                                     std::remove_cvref_t<decltype(shape)>,
                                     geometry::triangle>) {
                return light_dist{pos, rotation, triangle_dist{shape}};
            } else {
                std::cerr << "[warn] unsupported shape in make_light_dist\n";
                return light_dist{pos, rotation,
                                  ellipsoid_dist{{geometry::vec3{1, 1, 1}}}};
            }
        },
        obj.shape);
}

struct bvh_mix_dist {
    BVH *bvh;

    template <class Rng>
    [[nodiscard]] inline geometry::vec3
    sample(Rng &rng, const geometry::vec3 &x,
           const geometry::vec3 &normal) const {
        auto dist = make_light_dist(bvh->obj_by_id(
            std::uniform_int_distribution<>(0, bvh->objects.size() - 1)(rng)));

        return dist.sample(rng, x, normal);
    }

    [[nodiscard]] inline float pdf(const geometry::vec3 &x,
                                   const geometry::vec3 &normal,
                                   const geometry::vec3 &dir) const {
        float res = 0;
        bvh->foreach_intersection(
            {x, dir}, EPS,
            [&normal, &x, &res](const geometry::ray &ray,
                                const ray_intersection_info &intr) {
                res += make_light_dist(*intr.obj).pdf_at(x, normal,
                                                         ray.at(intr.t));
            });
        return res / bvh->objects.size();
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

struct RaytracerStaticContext {
    const Scene &scene;
    BVH scene_bvh;
    BVH light_bvh;
    std::vector<const geometry::Object *> plains;
    dist_t dir_dist;

    RaytracerStaticContext(const Scene &scene) : scene(scene) {
        scene_bvh = BVH::build(
            std::span(scene.objects), [](const geometry::Object &obj) {
                return !std::holds_alternative<geometry::plane>(obj.shape);
            });
        light_bvh = BVH::build(
            std::span(scene.objects), [](const geometry::Object &obj) {
                return !std::holds_alternative<geometry::plane>(obj.shape) &&
                       obj.emission() != geometry::color3{0, 0, 0};
            });

        dist_t diffuse_dist = cosine_dist();
        if (!light_bvh.objects.empty()) {
            diffuse_dist = mix_dist{{diffuse_dist, bvh_mix_dist{&light_bvh}}};
        }
        for (auto &obj : scene.objects) {
            if (std::holds_alternative<geometry::plane>(obj.shape)) {
                plains.push_back(&obj);
            }
        }
        dir_dist = diffuse_dist;
    }
};
struct RaytracerThreadContext {
    const RaytracerStaticContext &static_context;
    std::minstd_rand rand_gen;
    dir_generator<std::minstd_rand> dir_gen;

    RaytracerThreadContext(const RaytracerStaticContext &static_context,
                           int rand_seed)
        : static_context(static_context), rand_gen(rand_seed),
          dir_gen({rand_gen, static_context.dir_dist}) {}

    [[nodiscard]] constexpr inline const Scene &scene() const {
        return static_context.scene;
    }

    [[nodiscard]] constexpr inline const BVH &scene_bvh() const {
        return static_context.scene_bvh;
    }

    [[nodiscard]] constexpr inline const BVH &light_bvh() const {
        return static_context.light_bvh;
    }

    [[nodiscard]] constexpr inline const std::vector<const geometry::Object *> &
    plains() const {
        return static_context.plains;
    }

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
    auto &camera = context.scene().camera;
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
    for (auto obj : context.plains()) {
        update_intersection(res, ray, *obj, intersect(ray, *obj, min_dst),
                            max_dst);
    }
    update_intersection(res, context.scene_bvh().intersect_ray(ray, min_dst),
                        max_dst);

    if (res.has_value() && res->t > max_dst)
        return std::nullopt;
    else
        return res;

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
    geometry::color3 res = context.scene().ambient_light;

    for (const auto &light : context.scene().lights) {
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
               : context.scene().bg_color;
}

[[nodiscard]] static inline geometry::color3
render_pixel(RaytracerThreadContext &context, int x, int y) {
    if constexpr (std::is_same_v<std::remove_const_t<decltype(ALGORITHM_TOKEN)>,
                                 monte_carlo_token_t>) {
        geometry::color3 res = {0, 0, 0};
        for (int s = 0; s < context.scene().samples; ++s) {
            auto ray = gen_ray(context, x, y);
            res += trace_ray(context, ray, context.scene().ray_depth);
        }
        return res / context.scene().samples;
    } else {
        auto ray = gen_ray(context.scene().camera, x, y);
        return trace_ray(context, ray, context.scene().ray_depth);
    }
}

inline void run_raytracer(const Scene &scene, Image &image) {
    if (scene.ray_depth == 0)
        return;

    RaytracerStaticContext ctx(scene);

    if constexpr (USE_MULTITHREADING) {
        auto worker_count = std::max(std::thread::hardware_concurrency(), 1u);
        std::vector<std::thread> workers;
        workers.reserve(worker_count);
        std::atomic_int next_span(0);
        int span_count = (image.data.size() + SPAN_SIZE - 1) / SPAN_SIZE;
        auto span = (image.data.size() + worker_count - 1) / worker_count;
        for (int i = 0; i < worker_count; ++i) {
            workers.push_back(std::thread([i, &ctx, &scene, &image, &next_span,
                                           &span_count]() {
                int span = -1;
                while ((span = next_span.fetch_add(1)) < span_count) {
                    RaytracerThreadContext context(ctx, span);
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
        RaytracerThreadContext context(ctx, 42);
        for (int y = 0; y < scene.camera.height; ++y) {
            for (int x = 0; x < scene.camera.width; ++x) {
                image.set_pixel(x, y, render_pixel(context, x, y));
            }
        }
    }
}

#endif // RAYTRACER_H