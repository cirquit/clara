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

        cs_1.add_observation(1, 2);
        cs_1.add_observation(1.5, 2.1);
        cs_1.update_state();
        double mle_01 = cs_1.maximum_likelihood_estimate(1,2);
        double mle_02 = cs_1.maximum_likelihood_estimate(1,1);

        std::cout << "mle_01: " << mle_01 << '\n'
                  << "mle_02: " << mle_02 << '\n';
        REQUIRE(true);
    }
}
