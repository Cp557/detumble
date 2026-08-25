#include <string>

#include <catch2/catch_test_macros.hpp>

#include "detumble/version.hpp"

TEST_CASE("the core library reports the project version") {
    REQUIRE(std::string{detumble::version()} == "0.1.0");
}
