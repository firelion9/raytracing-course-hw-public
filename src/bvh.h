#ifndef BVH_H
#define BVH_H

#include "geometry.h"
#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <utility>
#include <variant>
#include <vector>

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

[[nodiscard]] constexpr static inline ray_intersection_info
to_intersection_info(const geometry::Object &obj, const geometry::ray &ray,
                     float t) {
    auto pos = ray.at(t);
    auto normal = obj.normal_at(pos);
    auto is_inside = geometry::dot(normal, ray.dir) > 0;
    return {is_inside ? -normal : normal, t, &obj, is_inside};
}

constexpr static inline void
update_intersection(ray_cast_res &res, const geometry::ray &ray,
                    const geometry::Object &obj,
                    const intersection_res &intersection, float max_dst) {
    if (intersection.empty())
        return;
    auto t = intersection.min();
    if (t > max_dst)
        return;

    if (!res.has_value() || res->t > t) {
        res = to_intersection_info(obj, ray, t);
    }
}

constexpr static inline void
update_intersection(ray_cast_res &res, const ray_cast_res &intersection,
                    float max_dst = INFINITY) {
    if (!intersection.has_value())
        return;
    auto t = intersection->t;
    if (t > max_dst)
        return;

    if (!res.has_value() || res->t > t) {
        res = intersection;
    }
}

[[nodiscard]] constexpr static inline std::optional<float>
intersect(const geometry::ray &ray, const geometry::aabb &box,
          float min_dst = INFINITY) {

    auto intrs1 = (box.vmin() - ray.start) / ray.dir;
    auto intrs2 = (box.vmax() - ray.start) / ray.dir;

    auto t_min = max_component(min(intrs1, intrs2));
    auto t_max = min_component(max(intrs1, intrs2));

    if (t_min <= t_max) {
        if (t_min >= min_dst)
            return t_min;
        if (t_max >= min_dst)
            return t_max;
    }

    return std::nullopt;
}

static constexpr std::uint32_t NO_CHILD =
    std::numeric_limits<std::uint32_t>::max();

struct BVHNode {
    geometry::aabb bounding_box;
    std::uint32_t left_child;
    std::uint32_t right_child;
    std::uint32_t obj_begin;
    std::uint32_t obj_end;
};

struct BVH {
    std::vector<const geometry::Object *> objects;
    std::vector<BVHNode> nodes;
    std::uint32_t root;

    [[nodiscard]] inline ray_cast_res intersect_ray(const geometry::ray &ray,
                                                    float min_dst) const {
        if (root == NO_CHILD)
            return {};
        return intersect_ray(ray, min_dst, root);
    }

    template <class Fn>
    constexpr inline void foreach_intersection(const geometry::ray &ray,
                                               float min_dst, Fn &&fn) const {
        if (root == NO_CHILD)
            return;
        foreach_intersection(ray, min_dst, std::forward<Fn>(fn), root);
    }

    [[nodiscard]] inline const geometry::Object &
    obj_by_id(std::uint32_t id) const {
        return *objects[id];
    }

    [[nodiscard]] inline ray_cast_res
    intersect_ray(const geometry::ray &ray, float min_dst,
                  std::uint32_t node_id) const {
        ray_cast_res intr;
        const BVHNode &node = nodes[node_id];
        for (std::uint32_t obj_id = node.obj_begin; obj_id < node.obj_end;
             ++obj_id) {
            update_intersection(intr, ray, obj_by_id(obj_id),
                                intersect(ray, obj_by_id(obj_id), min_dst),
                                INFINITY);
        }
        auto d_left =
            node.left_child == NO_CHILD
                ? std::nullopt
                : intersect(ray, nodes[node.left_child].bounding_box, min_dst);
        auto d_right =
            node.right_child == NO_CHILD
                ? std::nullopt
                : intersect(ray, nodes[node.right_child].bounding_box, min_dst);
        if (d_left.has_value() && d_right.has_value()) {
            std::uint32_t id1 = node.left_child;
            std::uint32_t id2 = node.right_child;
            if (d_left > d_right) {
                std::swap(id1, id2);
                std::swap(d_left, d_right);
            }
            update_intersection(intr, intersect_ray(ray, min_dst, id1));
            if (!intr.has_value() || intr->t > d_right) {
                update_intersection(intr, intersect_ray(ray, min_dst, id2));
            }
        } else {
            if (d_left.has_value()) {
                update_intersection(
                    intr, intersect_ray(ray, min_dst, node.left_child));
            }
            if (d_right.has_value()) {
                update_intersection(
                    intr, intersect_ray(ray, min_dst, node.right_child));
            }
        }
        return intr;
    }

    template <class Fn>
    inline void foreach_intersection(const geometry::ray &ray, float min_dst,
                                     Fn &&fn, std::uint32_t node_id) const {
        const BVHNode &node = nodes[node_id];
        for (std::uint32_t obj_id = node.obj_begin; obj_id < node.obj_end;
             ++obj_id) {
            for (auto t : intersect(ray, obj_by_id(obj_id), min_dst)) {
                fn(ray, to_intersection_info(obj_by_id(obj_id), ray, t));
            }
        }

        if (node.left_child != NO_CHILD &&
            intersect(ray, nodes[node.left_child].bounding_box, min_dst)
                .has_value()) {
            foreach_intersection(ray, min_dst, fn, node.left_child);
        }

        if (node.right_child != NO_CHILD &&
            intersect(ray, nodes[node.right_child].bounding_box, min_dst)
                .has_value()) {
            foreach_intersection(ray, min_dst, fn, node.right_child);
        }
    }

    template <int coord>
    [[nodiscard]] constexpr static inline bool
    obj_cmp(const geometry::Object *left, const geometry::Object *right) {
        return left->center().val[coord] < right->center().val[coord];
    }

    [[nodiscard]] static inline auto
    split_node(const std::span<const geometry::Object *> &objs,
               const geometry::aabb &box, std::vector<float> &tmp_pref,
               std::vector<float> &tmp_suf) {
        geometry::vec3 diag = box.diag();
        auto cmp = diag.x() >= diag.y() && diag.x() >= diag.z()
                       ? BVH::obj_cmp<0>
                   : diag.y() >= diag.z() ? BVH::obj_cmp<1>
                                          : BVH::obj_cmp<2>;

        std::sort(objs.begin(), objs.end(), cmp);

        tmp_pref.clear();
        tmp_suf.clear();

        geometry::aabb acc_box;
        tmp_pref.push_back(acc_box.surface_area());
        for (int i = 0; i < objs.size(); ++i) {
            auto &obj = *objs[i];
            acc_box.extend(obj.bounding_box());
            tmp_pref.push_back(acc_box.surface_area());
        }

        acc_box = geometry::aabb();
        tmp_suf.push_back(acc_box.surface_area());
        for (int i = objs.size() - 1; i >= 0; --i) {
            auto &obj = *objs[i];
            acc_box.extend(obj.bounding_box());
            tmp_suf.push_back(acc_box.surface_area());
        }

        auto split_ptr = objs.end();
        float split_score = objs.size() * acc_box.surface_area();

        for (int i = 1; i < objs.size(); ++i) {
            float score = i * tmp_pref[i + 1] +
                          (objs.size() - i) * tmp_suf[objs.size() - i];

            if (score < split_score) {
                split_score = score;
                split_ptr = objs.begin() + i;
            }
        }

        return split_ptr;
    }

    [[nodiscard]] constexpr static inline geometry::aabb
    bounding_box_of(const std::span<const geometry::Object *> &objs) {
        geometry::aabb res;
        for (auto obj : objs)
            res.extend(obj->bounding_box());
        return res;
    }

    [[nodiscard]] static inline std::uint32_t
    build_node(std::vector<BVHNode> &nodes, std::uint32_t objs_offset,
               const std::span<const geometry::Object *> &objs,
               const geometry::aabb &box, std::uint32_t min_node_size,
               std::uint32_t max_depth, std::vector<float> &tmp1,
               std::vector<float> &tmp2) {

        auto no_split = [&nodes, &box, &objs_offset, &objs]() {
            nodes.push_back({box, NO_CHILD, NO_CHILD, objs_offset,
                             static_cast<uint32_t>(objs_offset + objs.size())});
            return nodes.size() - 1;
        };

        if (max_depth == 0) {
            return no_split();
        } else {
            auto mid = split_node(objs, box, tmp1, tmp2);
            auto left_span = std::span(objs.begin(), mid);
            auto right_span = std::span(mid, objs.end());

            if (left_span.empty() || right_span.empty() ||
                (left_span.size() < min_node_size &&
                 right_span.size() < min_node_size)) {
                return no_split();
            } else {
                geometry::aabb left_bd = bounding_box_of(left_span);
                geometry::aabb right_bd = bounding_box_of(right_span);

                std::uint32_t idx = nodes.size();
                nodes.push_back({box, NO_CHILD, NO_CHILD, 0, 0});
                std::uint32_t left = build_node(
                    nodes, objs_offset, left_span, bounding_box_of(left_span),
                    min_node_size, max_depth - 1, tmp1, tmp2);
                std::uint32_t right =
                    build_node(nodes, objs_offset + left_span.size(),
                               right_span, bounding_box_of(right_span),
                               min_node_size, max_depth - 1, tmp1, tmp2);
                nodes[idx].left_child = left;
                nodes[idx].right_child = right;

                return idx;
            }
        }
    }

    template <class Pred>
    [[nodiscard]] static inline BVH
    build(const std::span<const geometry::Object> &objs, Pred &&pred,
          std::uint32_t min_node_size = 4, std::uint32_t max_depth = 64) {
        BVH res;
        if (objs.size() == 0) {
            res.root = NO_CHILD;
            return res;
        }
        res.objects.reserve(objs.size());
        for (auto &obj : objs) {
            if (pred(obj)) {
                res.objects.push_back(&obj);
            }
        }

        std::vector<float> tmp1, tmp2;
        tmp1.reserve(objs.size() + 1);
        tmp2.reserve(objs.size() + 1);

        res.root = build_node(res.nodes, 0, std::span(res.objects),
                              bounding_box_of(std::span(res.objects)),
                              min_node_size, max_depth, tmp1, tmp2);

        return res;
    }
};

#endif // BVH_H
