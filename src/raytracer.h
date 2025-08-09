#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "bvh.h"
#include "config.h"
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
#include <variant>
#include <vector>

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
                                       const geometry::color3 &v) {
    return out << "(" << v.r() << ", " << v.g() << ", " << v.b() << ")";
}

static inline std::ostream &operator<<(std::ostream &out,
                                       const geometry::ray &v) {
    return out << v.start << "-" << v.dir << "->";
}

// endregion

// region internal structures

struct cosine_dist;
struct triangle_dist;
using light_dist = triangle_dist;
struct mix_dist;
struct bvh_mix_dist;
using dist_t = std::variant<cosine_dist, light_dist, mix_dist, bvh_mix_dist>;

[[nodiscard]] constexpr static inline std::pair<float, float>
intersect_ray_sphere(const geometry::ray &ray, float r) {
    intersection_res res;

    float a = geometry::dot(ray.dir / r, ray.dir / r);
    float hb = geometry::dot(ray.start / r, ray.dir / r);
    float c = geometry::dot(ray.start / r, ray.start / r) - 1;
    float hd2 = hb * hb - a * c;
    if (hd2 < 0)
        return {0, 0};
    float hd = std::sqrt(hd2);

    float t1 = (-hb - hd) / a;
    float t2 = (-hb + hd) / a;

    return {t1, t2};
}

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
        return sample(rng);
    }

    template <class Rng>
    [[nodiscard]] inline geometry::vec3 sample(Rng &rng) const {
        auto z_dist = std::uniform_real_distribution<float>(-1.0f, 1.0f);
        auto angle_dist = std::uniform_real_distribution<float>(
            0.0f, 2 * std::numbers::pi_v<float>);

        float z = z_dist(rng);
        float co_z = std::sqrt(std::max(0.0f, 1 - z * z));
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
        auto dir = normal + sphere_uniform_dist().sample(rng);
        return geometry::norm(dir);
    }

    [[nodiscard]] inline float pdf(const geometry::vec3 &x,
                                   const geometry::vec3 &normal,
                                   const geometry::vec3 &dir) const {
        return std::max(geometry::dot(normal, dir) / std::numbers::pi_v<float>,
                        0.0f);
    }
};

[[nodiscard]] constexpr inline geometry::vec3
halfway(const geometry::vec3 &in_dir, const geometry::vec3 &out_dir) {
    return geometry::norm(out_dir - in_dir);
}

#define _ << " " <<
struct VNDF_dist {
    float roughness = 1.0;

    template <class Rng>
    [[nodiscard]] inline geometry::vec3
    sample(Rng &rng, const geometry::vec3 &in_dir,
           const geometry::vec3 &normal) const {
        auto dist = std::uniform_real_distribution<float>(0, 1);
        auto nx = choose_local_x(normal);
        auto ny = geometry::crs(normal, nx);

        auto v = -geometry::vec3(geometry::dot(nx, in_dir),
                                 geometry::dot(ny, in_dir),
                                 geometry::dot(normal, in_dir));

        auto vh = geometry::norm(geometry::vec3(roughness, roughness, 1) * v);
        float lensq = vh.xy().len2();
        auto T1 = lensq > 0
                      ? geometry::vec3(-vh.y(), vh.x(), 0) / std::sqrt(lensq)
                      : geometry::vec3(1, 0, 0);
        auto T2 = geometry::crs(vh, T1);
        float r = std::sqrt(dist(rng));
        float phi = 2.0f * std::numbers::pi_v<float> * dist(rng);
        float t1 = r * cos(phi);
        float t2 = r * sin(phi);
        float s = 0.5f * (1.0f + vh.z());
        t2 = (1.0f - s) * std::sqrt(1.0f - t1 * t1) + s * t2;

        auto nh = t1 * T1 + t2 * T2 +
                  std::sqrt(std::max(0.0f, 1.0f - t1 * t1 - t2 * t2)) * vh;

        auto ne = geometry::norm(geometry::vec3(roughness * nh.x(),
                                                roughness * nh.y(),
                                                std::max<float>(0.0f, nh.z())));
        auto res_n =
            geometry::norm(nx * ne.x() + ny * ne.y() + normal * ne.z());
        auto res = geometry::reflect(res_n, in_dir);
        return res;
    }

    [[nodiscard]] inline float pdf(const geometry::vec3 &in_dir,
                                   const geometry::vec3 &normal,
                                   const geometry::vec3 &dir) const {
        auto nx = choose_local_x(normal);
        auto ny = geometry::crs(normal, nx);

        auto v = -geometry::vec3(geometry::dot(nx, in_dir),
                                 geometry::dot(ny, in_dir),
                                 geometry::dot(normal, in_dir));

        if (std::abs(v.z()) < EPS) {
            return 0.0f;
        }

        auto nv = halfway(in_dir, dir);
        auto n = geometry::vec3(geometry::dot(nx, nv), geometry::dot(ny, nv),
                                geometry::dot(normal, nv));
        float lambda =
            (-1 +
             std::sqrt(1 +
                       (v.xy() * geometry::vec2(roughness, roughness)).len2() /
                           pow2(v.z()))) /
            2;
        float g1 = 1 / (1 + lambda);
        float dn = 1 / std::numbers::pi_v<float> / roughness / roughness /
                   pow2((n / geometry::vec3(roughness, roughness, 1)).len2());
        float dv = g1 * std::max(0.0f, geometry::dot(v, n)) * dn / v.z();
        return dv / 4 / geometry::dot(v, n);
    }

    [[nodiscard]] static constexpr inline geometry::vec3
    choose_local_x(const geometry::vec3 &n) {
        geometry::vec3 res{1, 1, 1};
        if (std::abs(n.x()) > 0.5f) {
            res.x() -= geometry::dot(res, n) / n.x();
        } else if (std::abs(n.y()) > 0.5f) {
            res.y() -= geometry::dot(res, n) / n.y();
        } else {
            res.z() -= geometry::dot(res, n) / n.z();
        }
        return geometry::norm(res);
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
        auto intr = intersect(ray, triangle, EPS);
        float res = 0;
        if (intr.has_value()) {
            auto y = ray.at(intr->z());
            res += light_surface_projection_multiplier(x, y, triangle.normal(),
                                                       dir);
        }
        return res / triangle.square();
    }

    [[nodiscard]] constexpr inline float pdf_at(const geometry::vec3 &x,
                                                const geometry::vec3 &normal,
                                                const geometry::vec3 &y) const {
        return light_surface_projection_multiplier(x, y, triangle.normal(),
                                                   geometry::norm(y - x)) /
               triangle.square();
    }
};

[[nodiscard]] constexpr inline float heaviside(float x) {
    return x > 0 ? 1 : 0;
}
[[nodiscard]] constexpr inline geometry::color3
conductor_fresnel(const geometry::color3 &f0, const geometry::color3 &bsdf,
                  float VdotH) {
    return bsdf * (f0 + (1 - f0) * pow<5>(1 - std::abs(VdotH)));
}

[[nodiscard]] constexpr inline geometry::color3
specular_brdf(const float alpha, const geometry::vec3 &in_dir,
              const geometry::vec3 &out_dir, const geometry::vec3 &normal) {
    auto h = halfway(in_dir, out_dir);
    auto d = pow2(alpha) * heaviside(geometry::dot(normal, h)) /
             std::numbers::pi_v<float> /
             pow2(pow2(geometry::dot(normal, h)) * (pow2(alpha) - 1) + 1);

    auto div1 =
        (std::abs(geometry::dot(normal, out_dir)) +
         std::sqrt(pow2(alpha) +
                   (1 - pow2(alpha)) * pow2(geometry::dot(normal, out_dir))));
    auto div2 =
        (std::abs(geometry::dot(normal, -in_dir)) +
         std::sqrt(pow2(alpha) +
                   (1 - pow2(alpha)) * pow2(geometry::dot(normal, -in_dir))));
    auto v = heaviside(geometry::dot(h, out_dir)) *
             heaviside(geometry::dot(h, -in_dir)) / div1 / div2;
    auto res = v * d;
    return {res, res, res};
}

[[nodiscard]] constexpr inline geometry::color3
diffuse_brdf(const geometry::color3 &color) {
    return color / std::numbers::pi_v<float>;
}

[[nodiscard]] constexpr inline geometry::color3
fresnel_mix(float ior, const geometry::color3 &base,
            const geometry::color3 &layer, float VdotH) {
    auto f0 = pow2((1 - ior) / (1 + ior));
    auto fr = f0 + (1 - f0) * pow<5>(1 - std::abs(VdotH));
    return base * (1 - fr) + layer * fr;
}

[[nodiscard]] constexpr inline geometry::color3
dielectric_brdf(const geometry::vec3 &in_dir, const geometry::vec3 &out_dir,
                const ray_intersection_info &intersection_info) {
    return fresnel_mix(
        intersection_info.ior, diffuse_brdf(intersection_info.color),
        specular_brdf(
            pow2(std::max(intersection_info.roughness, MIN_ROUGHNESS)), in_dir,
            out_dir, intersection_info.shading_normal),
        geometry::dot(-in_dir, halfway(in_dir, out_dir)));
}

[[nodiscard]] constexpr inline geometry::color3
metallic_brdf(const geometry::vec3 &in_dir, const geometry::vec3 &out_dir,
              const ray_intersection_info &intersection_info) {
    return conductor_fresnel(
        intersection_info.color,
        specular_brdf(
            pow2(std::max(intersection_info.roughness, MIN_ROUGHNESS)), in_dir,
            out_dir, intersection_info.shading_normal),
        geometry::dot(-in_dir, halfway(in_dir, out_dir)));
}

[[nodiscard]] constexpr inline geometry::color3
pbr_brdf(const geometry::vec3 &in_dir, const geometry::vec3 &out_dir,
         const ray_intersection_info &intersection_info) {
    geometry::color3 res = 0;
    if (intersection_info.metallic < 1) {
        res += (1 - intersection_info.metallic) *
               dielectric_brdf(in_dir, out_dir, intersection_info);
    }
    if (intersection_info.metallic > 0) {
        res += intersection_info.metallic *
               metallic_brdf(in_dir, out_dir, intersection_info);
    }
    return res;
}

[[nodiscard]] static constexpr inline light_dist
make_light_dist(const geometry::Object &obj) {
    return triangle_dist{obj.shape};
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
    dist_t dir_dist;

    RaytracerStaticContext(const Scene &scene) : scene(scene) {
        scene_bvh =
            BVH::build(std::span(scene.objects),
                       [](const geometry::Object &obj) { return true; });
        light_bvh = BVH::build(
            std::span(scene.objects), [](const geometry::Object &obj) {
                return obj.material.emission != geometry::color3{0, 0, 0};
            });

        dist_t diffuse_dist = cosine_dist();
        if (!light_bvh.objects.empty()) {
            diffuse_dist = mix_dist{{diffuse_dist, bvh_mix_dist{&light_bvh}}};
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
        float co_z = std::sqrt(std::max(0.0f, 1 - z * z));
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

    ray_cast_res res = context.scene_bvh().intersect_ray(ray, min_dst);

    if (res.has_value() && res->t > max_dst)
        return std::nullopt;
    else
        return res;

    return res;
}

[[nodiscard]] static inline geometry::color3
shade(RaytracerThreadContext &context, const geometry::ray &ray,
      const ray_intersection_info &intersection_info, unsigned max_depth) {
    auto pos = ray.at(intersection_info.t);
    auto &material = intersection_info.obj->material;
    auto VNDF_dst = VNDF_dist{std::max(material.roughness, MIN_ROUGHNESS)};
    auto dir = context.coin(VNDF_factor)
                   ? VNDF_dst.sample(context.rand_gen, ray.dir,
                                     intersection_info.shading_normal)
                   : context.dir_gen.sample(pos, intersection_info.normal);
    if (std::isnan(dir.x()) || std::isnan(dir.y()) || std::isnan(dir.z())) {
        return intersection_info.emission;
    }
    auto VNDF_p = VNDF_dst.pdf(ray.dir, intersection_info.shading_normal, dir);
    auto MIS_p = context.dir_gen.pdf(pos, intersection_info.normal, dir);
    auto p = VNDF_factor * VNDF_p + (1 - VNDF_factor) * MIS_p;

    if (p < EPS) {
        return intersection_info.emission;
    }

    auto scl =
        pbr_brdf(ray.dir, dir, intersection_info) / p *
        std::max(0.0f, geometry::dot(dir, intersection_info.shading_normal));

    if (scl.len2() == 0.0f) {
        return intersection_info.emission;
    }

    auto clr = trace_ray(context, {pos, dir}, max_depth) * scl;

    return intersection_info.emission + clr;
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
               : context.scene().bg_at(ray.dir);
}

[[nodiscard]] static inline geometry::color3
sanitize_nans(geometry::color3 clr) {
    if (std::isnan(clr.r()))
        clr.r() = 0;
    if (std::isnan(clr.g()))
        clr.g() = 0;
    if (std::isnan(clr.b()))
        clr.b() = 0;
    return clr;
}

[[nodiscard]] static inline geometry::color3
render_pixel(RaytracerThreadContext &context, int x, int y) {
    geometry::color3 res = {0, 0, 0};
    for (int s = 0; s < context.scene().samples; ++s) {
        auto ray = gen_ray(context, x, y);
        res +=
            sanitize_nans(trace_ray(context, ray, context.scene().ray_depth));
    }
    return res / context.scene().samples;
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
                    std::printf("%d/%d     \r", span, span_count);
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