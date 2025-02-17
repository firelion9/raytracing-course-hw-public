#ifndef IMAGE_H
#define IMAGE_H

#include "geometry.h"
#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

struct Image {
    int width, height;

    std::vector<geometry::color3> data;

    inline Image(int width, int height, geometry::color3 bg = {0, 0, 0})
        : width(width), height(height),
          data(std::vector<geometry::color3>(width * height, bg)) {
        if (width <= 0 || height <= 0) {
            throw std::runtime_error("Illegal image size" +
                                     std::to_string(width) + "x" +
                                     std::to_string(height));
        }
    }
    inline Image(const Image &) = default;
    inline Image(Image &&) = default;

    inline void write(std::ostream &out) {
        out << "P6\n" << width << " " << height << "\n" << "255\n";
        for (auto &clr : data) {
            const auto pixel = convert_color(clr);
            out.write(reinterpret_cast<const char *>(&pixel),
                      sizeof(convert_color(clr)));
        }
    }

    inline geometry::color3& at(int x, int y) {
        return data[x + y * width];
    }

  private:
    static inline std::array<uint8_t, 3>
    convert_color(const geometry::color3 &clr) {
        return {
            static_cast<uint8_t>(clr.r() * 255),
            static_cast<uint8_t>(clr.g() * 255),
            static_cast<uint8_t>(clr.b() * 255),
        };
    }
};

#endif // IMAGE_H
