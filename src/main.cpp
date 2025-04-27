#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <filesystem>

#include "image.h"
#include "raytracer.h"
#include "scene.h"


int main(int argc, char **argv) try {
    if (argc < 3) {
        std::cerr << "Too few arguments: expected 2, got " << argc - 1
                  << std::endl;
        return EXIT_FAILURE;
    }

    Scene scene = parse_gltf_scene(std::filesystem::path(argv[1]));
    Image img(scene.camera.width, scene.camera.height, scene.bg_color);

    run_raytracer(scene, img);

    {
        std::filesystem::path out_path = argv[2];
        std::filesystem::create_directories(out_path.parent_path());
        std::ofstream out(out_path, std::ios::binary);
        img.write(out);
    }

} catch (std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    return EXIT_FAILURE;
}
