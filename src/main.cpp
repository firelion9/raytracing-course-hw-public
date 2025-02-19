#include <fstream>
#include <iostream>
#include <stdexcept>

#include "image.h"
#include "raytracer.h"
#include "scene.h"


int main(int argc, char **argv) try {
    if (argc < 3) {
        std::cerr << "Too few arguments: expected 2, got " << argc - 1
                  << std::endl;
        return EXIT_FAILURE;
    }

    Scene scene;
    {
        std::ifstream in((argv[1]));
        scene = Scene::parse(in);
    }
    Image img(scene.camera.width, scene.camera.height, scene.bg_color);

    run_raytracer(scene, img);

    {
        std::ofstream out(argv[2], std::ios::binary);
        img.write(out);
    }

} catch (std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    return EXIT_FAILURE;
}
