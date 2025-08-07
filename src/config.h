#ifndef CONFIG_H
#define CONFIG_H

#include <cstddef>

constexpr bool USE_MULTITHREADING = true;
constexpr size_t SPAN_SIZE = 256;

constexpr float EPS = 1e-4;
constexpr float MIN_ROUGHNESS = 0.04f;

constexpr bool USE_TEXTURES = true;
constexpr bool USE_WHITE_BG = true;
constexpr bool USE_ENV_MAP = false;
constexpr char ENV_MAP_PATH[] = "env.hdr";
#endif // CONFIG_H
