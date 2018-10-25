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


void print_prob_pdf(std::string name, clara::cone_state<double> & cs_1, double x_cs_1, 
                                      clara::cone_state<double> & cs_2, double x_cs_2,
                                      clara::cone_state<double> & cs_3, double x_cs_3, double x_sum, std::tuple<double, double> x)
{
    std::cout << "# " << name << ":" << '\n'
              << "    cs_1: " << x_cs_1 / x_sum << ", pdf: " << cs_1.pdf(std::get<0>(x), std::get<1>(x)) << '\n'
              << "    cs_2: " << x_cs_2 / x_sum << ", pdf: " << cs_2.pdf(std::get<0>(x), std::get<1>(x)) << '\n'
              << "    cs_3: " << x_cs_3 / x_sum << ", pdf: " << cs_3.pdf(std::get<0>(x), std::get<1>(x)) << '\n';
}


TEST_CASE("cone_state.h", "[cone_state]") {

    // SECTION("testing getter, setter and modification flags for the cone_state class with different integral types") {
    //     cone_state_test<double>();
    //     cone_state_test<int>();
    //     cone_state_test<std::uint8_t>();
    // }

    SECTION("Testing if the simple cluster tools like mean and covariance working"){
        clara::cone_state<double> cs_1;
        clara::cone_state<double> cs_2;
        clara::cone_state<double> cs_3;

        cs_1.add_observation(0.5, 0.5, 0.5, 0.5);
        cs_1.add_observation(1.0, 0.5, 1.0, 0.5);
        cs_1.add_observation(1.0, 1.0, 1.0, 1.0);
        cs_1.add_observation(0.5, 1.5, 0.5, 1.5);
        cs_1.update_state();

        // mean
        REQUIRE(0.75 == Approx(cs_1._mean_vec[0]).epsilon(0.01));
        REQUIRE(0.875 == Approx(cs_1._mean_vec[1]).epsilon(0.01));
        // covariance 
        REQUIRE(0.063 == Approx(cs_1._cov_mat[0]).epsilon(0.01));
        REQUIRE(-0.031 == Approx(cs_1._cov_mat[1]).epsilon(0.01));
        REQUIRE(-0.031 == Approx(cs_1._cov_mat[2]).epsilon(0.01));
        REQUIRE(0.172 == Approx(cs_1._cov_mat[3]).epsilon(0.01));

        cs_2.add_observation(2.0, 4.5, 2.0, 4.5);
        cs_2.add_observation(2.5, 3.5, 2.5, 3.5);
        cs_2.add_observation(3.0, 5.5, 3.0, 5.5);
        cs_2.update_state();

        // mean
        REQUIRE(2.5 == Approx(cs_2._mean_vec[0]).epsilon(0.01));
        REQUIRE(4.5 == Approx(cs_2._mean_vec[1]).epsilon(0.01));
        // covariance 
        REQUIRE(0.167 == Approx(cs_2._cov_mat[0]).epsilon(0.01));
        REQUIRE(0.167 == Approx(cs_2._cov_mat[1]).epsilon(0.01));
        REQUIRE(0.167 == Approx(cs_2._cov_mat[2]).epsilon(0.01));
        REQUIRE(0.667 == Approx(cs_2._cov_mat[3]).epsilon(0.01));

        cs_3.add_observation(4.5, 1.5, 4.5, 1.5);
        cs_3.add_observation(4.5, 2.0, 4.5, 2.0);
        cs_3.add_observation(4.5, 2.5, 4.5, 2.5);
        cs_3.add_observation(4.0, 2.0, 4.0, 2.0);
        cs_3.add_observation(5.0, 1.0, 5.0, 1.0);
        cs_3.update_state();

        // mean
        REQUIRE(4.5 == Approx(cs_3._mean_vec[0]).epsilon(0.01));
        REQUIRE(1.8 == Approx(cs_3._mean_vec[1]).epsilon(0.01));
        // covariance 
        REQUIRE(0.1 == Approx(cs_3._cov_mat[0]).epsilon(0.01));
        REQUIRE(-0.1 == Approx(cs_3._cov_mat[1]).epsilon(0.01));
        REQUIRE(-0.1 == Approx(cs_3._cov_mat[2]).epsilon(0.01));
        REQUIRE(0.26 == Approx(cs_3._cov_mat[3]).epsilon(0.01));

        std::tuple<double, double> x1 { 1.5, 2.0 };
        std::tuple<double, double> x2 { 4.0, 4.0 };
        std::tuple<double, double> x3 { 5.0, 1.5 };
        std::tuple<double, double> x4 { 1.5, 0.5 };
        std::tuple<double, double> x5 { 2.0, 5.5 };
        std::tuple<double, double> x6 { 8.0, 7.0 };

        double cs_1_ob_count = cs_1.get_observations_size();
        double cs_2_ob_count = cs_2.get_observations_size();
        double cs_3_ob_count = cs_3.get_observations_size();
        double ob_count_sum  = cs_1_ob_count + cs_2_ob_count + cs_3_ob_count;
        double cs_1_weight   = cs_1_ob_count / ob_count_sum;
        double cs_2_weight   = cs_2_ob_count / ob_count_sum;
        double cs_3_weight   = cs_3_ob_count / ob_count_sum;

        REQUIRE(0.33 == Approx(cs_1_weight).epsilon(0.01));
        REQUIRE(0.25 == Approx(cs_2_weight).epsilon(0.01));
        REQUIRE(0.417 == Approx(cs_3_weight).epsilon(0.01));

        double x1_cs_1 = cs_1_weight * cs_1.pdf(std::get<0>(x1), std::get<1>(x1));
        double x1_cs_2 = cs_2_weight * cs_2.pdf(std::get<0>(x1), std::get<1>(x1));
        double x1_cs_3 = cs_3_weight * cs_3.pdf(std::get<0>(x1), std::get<1>(x1));
        double x1_sum = x1_cs_1 + x1_cs_2 + x1_cs_3;

        REQUIRE(0.006 == Approx(x1_cs_1 / x1_sum).epsilon(0.0005));
        REQUIRE(0.994 == Approx(x1_cs_2 / x1_sum).epsilon(0.0005));
        REQUIRE(0.0   == Approx(x1_cs_3 / x1_sum).epsilon(0.0005));

        // print_prob_pdf("x1", cs_1, x1_cs_1, cs_2, x1_cs_2, cs_3, x1_cs_3, x1_sum, x1);

        double x2_cs_1 = cs_1_weight * cs_1.pdf(std::get<0>(x2), std::get<1>(x2));
        double x2_cs_2 = cs_2_weight * cs_2.pdf(std::get<0>(x2), std::get<1>(x2));
        double x2_cs_3 = cs_3_weight * cs_3.pdf(std::get<0>(x2), std::get<1>(x2));
        double x2_sum = x2_cs_1 + x2_cs_2 + x2_cs_3;

        REQUIRE(0.0   == Approx(x2_cs_1 / x2_sum).epsilon(0.0005));
        REQUIRE(0.141 == Approx(x2_cs_2 / x2_sum).epsilon(0.0005));
        REQUIRE(0.859 == Approx(x2_cs_3 / x2_sum).epsilon(0.0005));

        // print_prob_pdf("x2", cs_2, x2_cs_1, cs_2, x2_cs_2, cs_3, x2_cs_3, x2_sum, x2);

        double x3_cs_1 = cs_1_weight * cs_1.pdf(std::get<0>(x3), std::get<1>(x3));
        double x3_cs_2 = cs_2_weight * cs_2.pdf(std::get<0>(x3), std::get<1>(x3));
        double x3_cs_3 = cs_3_weight * cs_3.pdf(std::get<0>(x3), std::get<1>(x3));
        double x3_sum = x3_cs_1 + x3_cs_2 + x3_cs_3;

        REQUIRE(0.0 == Approx(x3_cs_1 / x3_sum).epsilon(0.0005));
        REQUIRE(0.0 == Approx(x3_cs_2 / x3_sum).epsilon(0.0005));
        REQUIRE(1.0 == Approx(x3_cs_3 / x3_sum).epsilon(0.0005));

        // print_prob_pdf("x3", cs_1, x3_cs_1, cs_2, x3_cs_2, cs_3, x3_cs_3, x3_sum, x3);

        double x4_cs_1 = cs_1_weight * cs_1.pdf(std::get<0>(x4), std::get<1>(x4));
        double x4_cs_2 = cs_2_weight * cs_2.pdf(std::get<0>(x4), std::get<1>(x4));
        double x4_cs_3 = cs_3_weight * cs_3.pdf(std::get<0>(x4), std::get<1>(x4));
        double x4_sum = x4_cs_1 + x4_cs_2 + x4_cs_3;

        REQUIRE(1.0 == Approx(x4_cs_1 / x4_sum).epsilon(0.0005));
        REQUIRE(0.0 == Approx(x4_cs_2 / x4_sum).epsilon(0.0005));
        REQUIRE(0.0 == Approx(x4_cs_3 / x4_sum).epsilon(0.0005));
        
        // print_prob_pdf("x4", cs_1, x4_cs_1, cs_2, x4_cs_2, cs_3, x4_cs_3, x4_sum, x4);

        double x5_cs_1 = cs_1_weight * cs_1.pdf(std::get<0>(x5), std::get<1>(x5));
        double x5_cs_2 = cs_2_weight * cs_2.pdf(std::get<0>(x5), std::get<1>(x5));
        double x5_cs_3 = cs_3_weight * cs_3.pdf(std::get<0>(x5), std::get<1>(x5));
        double x5_sum = x5_cs_1 + x5_cs_2 + x5_cs_3;

        REQUIRE(0.0 == Approx(x5_cs_1 / x5_sum).epsilon(0.0005));
        REQUIRE(1.0 == Approx(x5_cs_2 / x5_sum).epsilon(0.0005));
        REQUIRE(0.0 == Approx(x5_cs_3 / x5_sum).epsilon(0.0005));

        // print_prob_pdf("x5", cs_1, x5_cs_1, cs_2, x5_cs_2, cs_3, x5_cs_3, x5_sum, x5);
    }
}