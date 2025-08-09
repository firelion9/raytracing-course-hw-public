#ifndef CONFIG_H
#define CONFIG_H

#include <array>
#include <cstddef>

constexpr bool USE_MULTITHREADING = true;
/** 
 * When [USE_MULTITHREADING] is true, resulting image is split into parts
 * of [SPAN_SIZE] pixels each (consecutive in row-major order).
 * Each of this parts is single task for a thread pool
 */
constexpr size_t SPAN_SIZE = 256;

constexpr float EPS = 1e-4;

constexpr unsigned DEFAULT_RAY_DEPTH = 8;

/** Roughness less than [MIN_ROUGHNESS] is treated as equal to [MIN_ROUGHNESS] */
constexpr float MIN_ROUGHNESS = 0.04f;

/** 
 * Weigh of VNDF in multiple importance sampling
 * (cosine-weighted and light sampling both have weight equal to $(1 - [VNDF_factor]) / 2$ ) 
 */
constexpr float VNDF_factor = 1 / 3.0f;

/** If false, only single-pixel textures are supported */
constexpr bool USE_TEXTURES = true;

// Next 3 constants setup environment map.
// [USE_WHITE_BG] = false => env map is black
// [USE_WHITE_BG] = true, [USE_ENV_MAP] = false => env map is white (intensity = 1.0)
// [USE_WHITE_BG] = true, [USE_ENV_MAP] = true and [ENV_MAP_PATH] set => 
//     => env map is RGB image from [ENV_MAP_PATH] (any that is supported by stb_image)
constexpr bool USE_WHITE_BG = false;
constexpr bool USE_ENV_MAP = false;
constexpr char ENV_MAP_PATH[] = "env.hdr";

// Setup extra light source (in camera local coordinates) 
constexpr bool ADD_LIGHT_TRIANGLE = false;
constexpr float LIGHT_TRIANGLE_INTENSITY = 10;
constexpr std::array<std::array<float, 3>, 3> LIGHT_TRIANGLE_RELATIVE_POS = {
    std::array<float, 3>{10, 0, -0.1},
    std::array<float, 3>{0, 10, -0.1},
    std::array<float, 3>{0, -10, -0.1},
};

#endif // CONFIG_H
