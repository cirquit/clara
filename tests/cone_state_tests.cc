// Copyright 2018 municHMotorsport e.V. <info@munichmotorsport.de>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <blaze/Math.h>
#include <vector>
#include <iostream>
#include <random>
#include <tuple>
#include <cstdint>
#include "catch.h"


#include "../library/cone_state.h"

// template<typename T>
// void cone_state_test()
// {
//     clara::cone_state<T> cs_1;
//     REQUIRE_FALSE(cs_1.is_modified());

//     T x = 1;
//     T y = 2;
//     std::tuple<T, T> coords { x, y };

//     cs_1.set_coords(x, y);

//     REQUIRE(cs_1.get_coords() == coords);
//     REQUIRE(cs_1.get_x() == x);
//     REQUIRE(cs_1.get_y() == y);
//     REQUIRE(cs_1.is_modified());
// }

TEST_CASE("cone_state.h", "[cone_state]") {

    // SECTION("testing getter, setter and modification flags for the cone_state class with different integral types") {
    //     cone_state_test<double>();
    //     cone_state_test<int>();
    //     cone_state_test<std::uint8_t>();
    // }

    SECTION("matrix stuff"){
        clara::cone_state<double> cs_1;
        clara::cone_state<double> cs_2;
        clara::cone_state<double> cs_3;

        cs_1.add_observation(0.5, 0.5);
        cs_1.add_observation(1.0, 0.5);
        cs_1.add_observation(1.0, 1.0);
        cs_1.add_observation(0.5, 1.5);
        cs_1.update_state();

        cs_2.add_observation(2.0, 4.5);
        cs_2.add_observation(2.5, 3.5);
        cs_2.add_observation(3.0, 5.5);
        cs_2.update_state();

        cs_3.add_observation(4.5, 1.5);
        cs_3.add_observation(4.5, 2.0);
        cs_3.add_observation(4.5, 2.5);
        cs_3.add_observation(4.0, 2.0);
        cs_3.add_observation(5.0, 1.0);
        cs_3.update_state();

        std::tuple<double, double> x1 { 1.5, 2.0 };
        std::tuple<double, double> x2 { 4.0, 4.0 };
        std::tuple<double, double> x3 { 5.0, 1.5 };
        std::tuple<double, double> x4 { 1.5, 0.5 };
        std::tuple<double, double> x5 { 2.0, 5.5 };

        double x1_cs_1 = cs_1.pdf(x1);
        double x1_cs_2 = cs_2.pdf(x1);
        double x1_cs_3 = cs_3.pdf(x1);
        double x1_sum = x1_cs_1 + x1_cs_2 + x1_cs_3;

        std::cout << "# x1:" << '\n'
                  << "    cs_1: " << x1_cs_1 / x1_sum << '\n'
                  << "    cs_2: " << x1_cs_2 / x1_sum << '\n'
                  << "    cs_3: " << x1_cs_3 / x1_sum << '\n';

        double x2_cs_1 = cs_1.pdf(x2);
        double x2_cs_2 = cs_2.pdf(x2);
        double x2_cs_3 = cs_3.pdf(x2);
        double x2_sum = x2_cs_1 + x2_cs_2 + x2_cs_3;

        std::cout << "# x2:" << '\n'
                  << "    cs_1: " << x2_cs_1 / x2_sum << '\n'
                  << "    cs_2: " << x2_cs_2 / x2_sum << '\n'
                  << "    cs_3: " << x2_cs_3 / x2_sum << '\n';

        double x3_cs_1 = cs_1.pdf(x3);
        double x3_cs_2 = cs_2.pdf(x3);
        double x3_cs_3 = cs_3.pdf(x3);
        double x3_sum = x3_cs_1 + x3_cs_2 + x3_cs_3;

        std::cout << "# x3:" << '\n'
                  << "    cs_1: " << x3_cs_1 / x3_sum << '\n'
                  << "    cs_2: " << x3_cs_2 / x3_sum << '\n'
                  << "    cs_3: " << x3_cs_3 / x3_sum << '\n';

        double x4_cs_1 = cs_1.pdf(x4);
        double x4_cs_2 = cs_2.pdf(x4);
        double x4_cs_3 = cs_3.pdf(x4);
        double x4_sum = x4_cs_1 + x4_cs_2 + x4_cs_3;

        std::cout << "# x4:" << '\n'
                  << "    cs_1: " << x4_cs_1 / x4_sum << '\n'
                  << "    cs_2: " << x4_cs_2 / x4_sum << '\n'
                  << "    cs_3: " << x4_cs_3 / x4_sum << '\n';

        double x5_cs_1 = cs_1.pdf(x5);
        double x5_cs_2 = cs_2.pdf(x5);
        double x5_cs_3 = cs_3.pdf(x5);
        double x5_sum = x5_cs_1 + x5_cs_2 + x5_cs_3;

        std::cout << "# x5:" << '\n'
                  << "    cs_1: " << x5_cs_1 / x5_sum << '\n'
                  << "    cs_2: " << x5_cs_2 / x5_sum << '\n'
                  << "    cs_3: " << x5_cs_3 / x5_sum << '\n';

        REQUIRE(true);
    }
}