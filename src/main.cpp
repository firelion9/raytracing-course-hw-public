#define STB_IMAGE_IMPLEMENTATION

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <filesystem>

#include "image.h"
#include "raytracer.h"
#include "scene.h"


int main(int argc, char **argv) try {
    if (argc < 6) {
        std::cerr << "Too few arguments: expected 6, got " << argc - 1
                  << std::endl;
        return EXIT_FAILURE;
    }

    unsigned width = std::strtol(argv[2], nullptr, 10);
    unsigned height = std::strtol(argv[3], nullptr, 10);
    unsigned samples = std::strtol(argv[4], nullptr, 10);

    Scene scene = parse_gltf_scene(std::filesystem::path(argv[1]), static_cast<float>(width) / height);
    scene.bg_color = {1, 1, 1};
    scene.camera.width = width;
    scene.camera.height = height;
    scene.samples = samples;
    Image img(width, height, scene.bg_color);

    run_raytracer(scene, img);

    {
        std::filesystem::path out_path = argv[5];
        std::filesystem::create_directories(out_path.parent_path());
        std::ofstream out(out_path, std::ios::binary);
        img.write(out);
    }

} catch (std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    return EXIT_FAILURE;
}
