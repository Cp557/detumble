#include <iostream>

#include "detumble/version.hpp"

int main() {
    std::cout << "Detumble " << detumble::version() << '\n';
    return 0;
}
