#ifndef SCENE_H
#define SCENE_H

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <optional>
#include <ostream>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "geometry.h"
#include "json.hpp"

using json = nlohmann::json;

constexpr unsigned DEFAULT_RAY_DEPTH = 8;

template <class T> static inline T typed_read(std::istream &in) {
    T res;
    in >> res;
    return res;
}

template <class FT, class... VS>
static inline void ensure_variant(std::variant<VS...> &variant,
                                  FT &&default_gen) {
    if (!std::holds_alternative<decltype(default_gen())>(variant)) {
        variant = default_gen();
    }
}

static inline void println(std::ostream &out) { out << "\n"; }

template <class Arg, class... Args>
static inline void println(std::ostream &out, Arg &&arg, Args &&...rest) {
    out << arg;
    println(out, std::forward<Args>(rest)...);
}

template <class... Args> static inline void warn(Args &&...args) {
    std::cerr << "WARN: ";
    println(std::cerr, std::forward<Args>(args)...);
}

struct Camera {
    unsigned width = 0;
    unsigned height = 0;
    geometry::vec3 position;
    geometry::vec3 right;
    geometry::vec3 up;
    geometry::vec3 forward;
    float fov_x = 0;

    [[nodiscard]] constexpr inline float fov_y() const {
        return atan(tan(fov_x / 2) * height / width) * 2;
    }
};

struct Scene {
    geometry::color3 bg_color;
    geometry::color3 ambient_light{1, 1, 1};
    unsigned ray_depth = 1;
    unsigned samples = 1;
    Camera camera;
    std::vector<geometry::Object> objects;
    std::vector<geometry::LightSource> lights;

    static inline Scene parse(std::istream &in) {
        Scene res;

        std::string buf;
        while (!in.eof()) {
            in >> buf;
            if (!in)
                break;

            // region general scene config
            if (buf == "RAY_DEPTH") {
                in >> res.ray_depth;
            } else if (buf == "SAMPLES") {
                in >> res.samples;
            } else if (buf == "BG_COLOR") {
                in >> res.bg_color;
            } else if (buf == "AMBIENT_LIGHT") {
                in >> res.ambient_light;
            }
            // endregion
            // region camera config
            else if (buf == "DIMENSIONS") {
                in >> res.camera.width >> res.camera.height;
            } else if (buf == "CAMERA_POSITION") {
                in >> res.camera.position;
            } else if (buf == "CAMERA_RIGHT") {
                in >> res.camera.right;
            } else if (buf == "CAMERA_UP") {
                in >> res.camera.up;
            } else if (buf == "CAMERA_FORWARD") {
                in >> res.camera.forward;
            } else if (buf == "CAMERA_FOV_X") {
                in >> res.camera.fov_x;
            }
            // endregion
            // region objects
            else if (buf == "NEW_PRIMITIVE") {
                res.objects.emplace_back();
            } else if (buf == "PLANE") {
                res.objects.back().shape =
                    geometry::plane{typed_read<geometry::vec3>(in)};
            } else if (buf == "ELLIPSOID") {
                res.objects.back().shape =
                    geometry::ellipsoid{typed_read<geometry::vec3>(in)};
            } else if (buf == "BOX") {
                res.objects.back().shape =
                    geometry::box{typed_read<geometry::vec3>(in)};
            } else if (buf == "TRIANGLE") {
                res.objects.back().shape =
                    geometry::triangle{typed_read<geometry::vec3>(in),
                                       typed_read<geometry::vec3>(in),
                                       typed_read<geometry::vec3>(in)};
            } else if (buf == "POSITION") {
                in >> res.objects.back().position;
            } else if (buf == "ROTATION") {
                in >> res.objects.back().rotation;
            } else if (buf == "COLOR") {
                in >> res.objects.back().color;
            }
            // endregion
            // region materials
            else if (buf == "METALLIC") {
                res.objects.back().material = geometry::metallic{};
            } else if (buf == "DIELECTRIC") {
                res.objects.back().material = geometry::dielectric{};
            } else if (buf == "IOR") {
                ensure_variant(res.objects.back().material, []() {
                    warn("Encounter IOR before DIELECTRIC or after another "
                         "material directive");
                    return geometry::dielectric{};
                });
                in >>
                    std::get<geometry::dielectric>(res.objects.back().material)
                        .ior;
            } else if (buf == "EMISSION") {
                std::visit([&in](auto &mat) { in >> mat.emission; },
                           res.objects.back().material);
            }
            // endregion
            // region lights
            else if (buf == "NEW_LIGHT") {
                res.lights.emplace_back();
            } else if (buf == "LIGHT_INTENSITY") {
                in >> res.lights.back().intensity;
            } else if (buf == "LIGHT_DIRECTION") {
                res.lights.back().light =
                    geometry::directed_light{typed_read<geometry::vec3>(in)};
            } else if (buf == "LIGHT_POSITION") {
                ensure_variant(res.lights.back().light,
                               []() { return geometry::point_light{}; });
                in >> std::get<geometry::point_light>(res.lights.back().light)
                          .position;
            } else if (buf == "LIGHT_ATTENUATION") {
                ensure_variant(res.lights.back().light,
                               []() { return geometry::point_light{}; });
                in >> std::get<geometry::point_light>(res.lights.back().light)
                          .attenuation;
            }
            // endregion
            else {
                warn("Unknow command while parsing scene: ", buf);
                std::getline(in, buf); // consume arguments
            }
        }

        if (!in.eof()) {
            throw std::runtime_error("Malformed input at pos " +
                                     std::to_string(in.tellg()));
        }

        return res;
    }
};

template <class Json>
static geometry::quaternion parse_quaternion(const Json &src) {
    return geometry::quaternion({src[0], src[1], src[2]}, src[3]);
}

template <class Json> static geometry::vec3 parse_vec3(const Json &src) {
    return geometry::vec3(src[0], src[1], src[2]);
}

template <class Json> static geometry::matrix4 parse_mat4(const Json &src) {
    return geometry::matrix4{{
        geometry::vec4{src[0], src[4], src[8], src[12]},
        {src[1], src[5], src[9], src[13]},
        {src[2], src[6], src[10], src[14]},
        {src[3], src[7], src[11], src[15]},
    }};
}

template <class Json> static geometry::color3 parse_color3(const Json &src) {
    return geometry::color3(src[0], src[1], src[2]);
}

template <class T, class Json>
static std::span<const T>
interpret_accessor(const Json &root,
                   const std::vector<std::vector<std::uint8_t>> &buffers,
                   size_t accessor_idx) {
    auto &accessor = root["accessors"][accessor_idx];
    int view_idx = accessor["bufferView"];
    auto &buffer_view = root["bufferViews"][view_idx];
    auto &buffer = buffers[buffer_view["buffer"]];
    std::size_t offset =
        buffer_view.contains("byteOffset")
            ? static_cast<std::size_t>(buffer_view["byteOffset"])
            : 0;
    std::size_t count = accessor["count"];
    return std::span<T>(reinterpret_cast<T *>(buffer.data() + offset), count);
}

struct unit_t {
} unit;

template <class Json>
static std::variant<std::span<const std::uint8_t>,
                    std::span<const std::uint16_t>,
                    std::span<const std::uint32_t>, unit_t>
load_indices(const Json &root,
             const std::vector<std::vector<std::uint8_t>> &buffers,
             std::optional<size_t> accessor_idx) {
    if (!accessor_idx.has_value())
        return unit;

    auto &accessor = root["accessors"][accessor_idx.value()];
    int view_idx = accessor["bufferView"];
    auto &buffer_view = root["bufferViews"][view_idx];
    auto &buffer = buffers[buffer_view["buffer"]];
    std::size_t acc_offset =
        accessor.contains("byteOffset")
            ? static_cast<std::size_t>(accessor["byteOffset"])
            : 0;
    std::size_t offset =
        (buffer_view.contains("byteOffset")
             ? static_cast<std::size_t>(buffer_view["byteOffset"])
             : 0) +
        acc_offset;
    std::size_t count = accessor["count"];
    int type = accessor["componentType"];
    switch (type) {
    case 5121:
        return std::span(
            reinterpret_cast<const std::uint8_t *>(buffer.data() + offset),
            count);
    case 5123:
        return std::span(
            reinterpret_cast<const std::uint16_t *>(buffer.data() + offset),
            count);

    case 5125:
        return std::span(
            reinterpret_cast<const std::uint32_t *>(buffer.data() + offset),
            count);

    default:
        throw std::runtime_error("illegal scalar type");
    }
}

static Scene parse_gltf_scene(const std::filesystem::path &gltf_path, float ar) {
    Scene res;
    res.ray_depth = DEFAULT_RAY_DEPTH;
    auto scene_struct = json::parse(std::ifstream(gltf_path));
    int scene_idx = scene_struct.contains("scene")
                        ? static_cast<int>(scene_struct["scene"])
                        : 0;
    auto &scene_info = scene_struct["scenes"][scene_idx];

    std::vector<std::vector<std::uint8_t>> buffers;
    for (auto &buf_info : scene_struct["buffers"]) {
        std::string uri = buf_info["uri"];
        std::ifstream buf_in(gltf_path.parent_path().append(uri),
                             std::ios::binary);
        buffers.emplace_back();
        auto &buf = buffers.back();
        buf.resize(buf_info["byteLength"]);
        buf_in.read(reinterpret_cast<char *>(buf.data()), buf.size());
    }

    std::function<void(int, const geometry::matrix4 &)> handle_node =
        [&res, &scene_struct, &buffers, &handle_node, ar](
            int node_idx, const geometry::matrix4 &parent_transform) {
            auto &node = scene_struct["nodes"][node_idx];
            geometry::quaternion rotation =
                node.contains("rotation") ? parse_quaternion(node["rotation"])
                                          : geometry::quaternion();
            geometry::vec3 translation = node.contains("translation")
                                             ? parse_vec3(node["translation"])
                                             : geometry::vec3(0, 0, 0);
            geometry::vec3 scale = node.contains("scale")
                                       ? parse_vec3(node["scale"])
                                       : geometry::vec3(1, 1, 1);
            geometry::matrix4 trs = node.contains("matrix")
                                        ? parse_mat4(node["matrix"])
                                        : geometry::matrix4::id();

            auto transform =
                parent_transform * trs *
                geometry::matrix4::transform(scale, rotation, translation);

            if (node.contains("camera")) {
                int camera_idx = node["camera"];
                auto camera = scene_struct["cameras"][camera_idx];
                auto perspective = camera["perspective"];
                float fov_y = perspective["yfov"];
                float aspect_ratio = perspective.contains("aspectRatio") ? static_cast<float>(perspective["aspectRatio"]) : ar;
                res.camera.position =
                    (transform * geometry::vec4(0, 0, 0, 1)).xyz();
                res.camera.forward =
                    geometry::norm(transform * geometry::vec4(0, 0, -1, 0))
                        .xyz();
                res.camera.up =
                    geometry::norm(transform * geometry::vec4(0, 1, 0, 0))
                        .xyz();
                res.camera.right =
                    geometry::norm(transform * geometry::vec4(1, 0, 0, 0))
                        .xyz();
                res.camera.fov_x = atan(tan(fov_y / 2) * aspect_ratio) * 2;
            }
            if (node.contains("mesh")) {
                int mesh_idx = node["mesh"];
                auto mesh = scene_struct["meshes"][mesh_idx];

                for (auto &primitive : mesh["primitives"]) {
                    int material_idx = primitive["material"];
                    auto material = scene_struct["materials"][material_idx];
                    geometry::color3 clr = geometry::color3(1, 1, 1);
                    geometry::color3 emission = geometry::color3(0, 0, 0);
                    if (material.contains("emissiveFactor")) {
                        emission = parse_color3(material["emissiveFactor"]);
                    }
                    if (material.contains(
                            "/extensions/KHR_materials_emissive_strength/emissiveStrength"_json_pointer)) {
                        emission *= static_cast<float>(
                            material
                                ["/extensions/KHR_materials_emissive_strength/emissiveStrength"_json_pointer]);
                    }
                    geometry::material mat = geometry::diffuse{emission};
                    if (material.contains("pbrMetallicRoughness")) {
                        auto &pbrMetallicRoughness =
                            material["pbrMetallicRoughness"];
                        if (pbrMetallicRoughness.contains("baseColorFactor")) {
                            auto &color =
                                pbrMetallicRoughness["baseColorFactor"];
                            if (color[3] < 1) {
                                mat = geometry::dielectric{emission, 1.5};
                            }
                            clr = parse_color3(color);
                        }
                        if (!pbrMetallicRoughness.contains("metallicFactor") ||
                            pbrMetallicRoughness["metallicFactor"] > 0) {
                            mat = geometry::metallic{emission};
                        }
                    }

                    int coord_accessor_idx =
                        primitive["attributes"]["POSITION"];
                    auto coords = interpret_accessor<const geometry::vec3>(
                        scene_struct, buffers, coord_accessor_idx);

                    auto indices = load_indices(scene_struct, buffers,
                                                primitive["indices"]);

                    auto get_index = [&indices](std::size_t idx) {
                        return std::visit(
                            [&idx](auto &idxs) {
                                if constexpr (std::is_assignable_v<
                                                  unit_t, decltype(idxs)>) {
                                    return static_cast<std::size_t>(idx);
                                } else {
                                    return static_cast<std::size_t>(idxs[idx]);
                                }
                            },
                            indices);
                    };
                    std::size_t cnt = std::visit(
                        [&coords](auto &idxs) {
                            if constexpr (std::is_assignable_v<
                                              unit_t, decltype(idxs)>) {
                                return coords.size();
                            } else {
                                return idxs.size();
                            }
                        },
                        indices);

                    int mode = primitive.contains("mode")
                                   ? static_cast<int>(primitive["mode"])
                                   : 4;

                    auto push_obj = [&res, &mat,
                                     &clr](const geometry::vec3 p1,
                                           const geometry::vec3 p2,
                                           const geometry::vec3 p3) {
                        res.objects.emplace_back();
                        auto &obj = res.objects.back();
                        obj.material = mat;
                        obj.color = clr;
                        obj.shape = geometry::triangle{p1, p2, p3};
                    };
                    switch (mode) {
                    case 4:
                        for (int i = 0; i < cnt; i += 3) {
                            auto p1 = transform.apply(coords[get_index(i)]);
                            auto p2 = transform.apply(coords[get_index(i + 1)]);
                            auto p3 = transform.apply(coords[get_index(i + 2)]);

                            push_obj(p1, p2, p3);
                        }
                        break;
                    case 5:
                        for (int i = 2; i < cnt; ++i) {
                            int off = i & 1;
                            auto p1 = transform.apply(coords[get_index(i - 2)]);
                            auto p2 = transform.apply(coords[get_index(i - 1 + off)]);
                            auto p3 = transform.apply(coords[get_index(i - off)]);

                            push_obj(p1, p2, p3);
                        }
                        break;
                    }
                }
            }
            if (node.contains("children")) {
                for (int child : node["children"]) {
                    handle_node(child, transform);
                }
            }
        };

    for (int node_idx : scene_info["nodes"]) {
        handle_node(node_idx, geometry::matrix4::id());
    }

    return res;
}

#endif // SCENE_H
