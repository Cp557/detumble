#include <iostream>

#include "detumble/version.hpp"

int main() {
    std::cout << "Detumble viewer " << detumble::version() << '\n';
    return 0;
}
