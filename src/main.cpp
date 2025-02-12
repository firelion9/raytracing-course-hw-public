#include <iostream>

int main(int argc, char **argv) {
    std::cout << "args " << (argc > 1 ? argv[1] : "<no arg>") << " " << (argc > 2 ? argv[2] : "<no arg>") << std::endl;
}
