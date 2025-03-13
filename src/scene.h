#ifndef SCENE_H
#define SCENE_H

#include <cmath>
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "geometry.h"

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
                std::visit([&in](auto &mat) { in >> mat.emission; }, res.objects.back().material);
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

#endif // SCENE_H
