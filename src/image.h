#ifndef IMAGE_H
#define IMAGE_H

#include "geometry.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

struct Image {
    using pixel_t = std::array<std::uint8_t, 3>;
    static_assert(sizeof(pixel_t) % alignof(pixel_t) == 0,
                  "pixel_t should not have any padding bytes");

    int width, height;
    std::vector<pixel_t> data;

    inline Image(int width, int height, geometry::color3 bg = {0, 0, 0})
        : width(width), height(height),
          data(std::vector<pixel_t>(width * height, convert_color(bg))) {
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
        out.write(reinterpret_cast<const char *>(data.data()),
                  sizeof(pixel_t) * data.size());
    }

    inline void set_pixel(int idx, const geometry::color3 &color) {
        data[idx] = convert_color(color);
    }

    inline void set_pixel(int x, int y, const geometry::color3 &color) {
        set_pixel(x + y * width, color);
    }

  private:
    static constexpr float GAMMA = 2.2f;

    [[nodiscard]] static inline geometry::color3
    aces_tonemap(const geometry::color3 &x) {
        const float a = 2.51f;
        const float b = 0.03f;
        const float c = 2.43f;
        const float d = 0.59f;
        const float e = 0.14f;
        return (x * (a * x + b)) / (x * (c * x + d) + e);
    }

    [[nodiscard]] static inline geometry::color3
    tone_mapping(const geometry::color3 &clr) {
        return geometry::pow(aces_tonemap(clr), 1 / GAMMA);
    }

    [[nodiscard]] static inline uint8_t discretize_channel(float x) {
        return static_cast<uint8_t>(
            std::round(std::clamp(x, 0.0f, 255.0f)));
    }

    [[nodiscard]] static inline std::array<uint8_t, 3>
    discretize_color(const geometry::color3 &clr) {
        return {
            discretize_channel(clr.r()),
            discretize_channel(clr.g()),
            discretize_channel(clr.b()),
        };
    }
    [[nodiscard]] static inline std::array<uint8_t, 3>
    convert_color(const geometry::color3 &clr) {
        return discretize_color(tone_mapping(clr) * 255);
    }
};

#endif // IMAGE_H
