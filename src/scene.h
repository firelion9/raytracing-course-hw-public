#ifndef SCENE_H
#define SCENE_H

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry.h"

struct Camera {
    int width = 0;
    int height = 0;
    geometry::vec3 position;
    geometry::vec3 right;
    geometry::vec3 up;
    geometry::vec3 forward;
    float fov_x = 0;

    [[nodiscard]] inline float fov_y() const {
        return atan(tan(fov_x / 2) * height / width) * 2;
    }
};

struct Scene {
    geometry::color3 bg_color;
    Camera camera;
    std::vector<geometry::Shape> objects;

    static inline Scene parse(std::istream &in) {
        Scene res;

        std::string buf;
        while (!in.eof()) {
            in >> buf;
            if (!in)
                break;

            if (buf == "DIMENSIONS") {
                in >> res.camera.width >> res.camera.height;
            } else if (buf == "BG_COLOR") {
                in >> res.bg_color;
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
            } else if (buf == "NEW_PRIMITIVE") {
                res.objects.emplace_back();
            } else if (buf == "PLANE") {
                res.objects.back().type = geometry::ShapeType::PLANE;
                in >> res.objects.back().prop;
            } else if (buf == "ELLIPSOID") {
                res.objects.back().type = geometry::ShapeType::ELLIPSOID;
                in >> res.objects.back().prop;
            } else if (buf == "BOX") {
                res.objects.back().type = geometry::ShapeType::BOX;
                in >> res.objects.back().prop;
            } else if (buf == "POSITION") {
                in >> res.objects.back().position;
            } else if (buf == "ROTATION") {
                in >> res.objects.back().rotation;
            } else if (buf == "COLOR") {
                auto &color = res.objects.back().color;
                in >> color.r() >> color.g() >> color.b();
                color.a() = 1.0;
            } else {
                throw std::runtime_error(
                    "Unknow command while parsing scene: " + buf);
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
