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

#include "catch.h"
#include <blaze/Math.h>
#include <iostream>
#include <random>
#include <vector>

#include "../library/util.h"

TEST_CASE( "util.h:angle-radians convertion", "[rad]" ) {

    REQUIRE( clara::util::angle_to_rad( 0 ) == 0.0 );
    REQUIRE( clara::util::angle_to_rad( 90 ) == Approx( 1.5708 ) );
    REQUIRE( clara::util::angle_to_rad( 180 ) == Approx( 3.1415926 ) );
    REQUIRE( clara::util::angle_to_rad( -90 ) == Approx( -1.5708 ) );

    REQUIRE( clara::util::rad_to_angle( 0 ) == 0.0 );
    REQUIRE( clara::util::rad_to_angle( 1.5708 ) == Approx( 90.0 ) );
    REQUIRE( clara::util::rad_to_angle( 3.1415926 ) == Approx( 180.0 ) );
    REQUIRE( clara::util::rad_to_angle( -1.5708 ) == Approx( -90 ) );

    for ( float f = 0.0; f < 360.1; f += 0.1 ) {
        REQUIRE( clara::util::rad_to_angle( clara::util::angle_to_rad( f ) ) == Approx( f ) );
    }
}

TEST_CASE( "util.h:custom iterator functions", "[iterators]" ) {

    SECTION( "for_each_" ) {
        std::vector< double > double_vec( 200, 0 );
        std::fill( double_vec.begin( ), double_vec.end( ), 1 );
        double for_each_sum = 0.0;
        clara::util::for_each_( double_vec, [&]( double & d ) { for_each_sum += d; } );
        REQUIRE( for_each_sum == 200 );
    };

    SECTION( "zipWith" ) {
        std::vector< double > double_vec( 200, 0 );
        std::fill( double_vec.begin( ), double_vec.end( ), 1 );
        std::vector< double > result_vec( 200, 0 );
        clara::util::zipWith( [&]( double x, double y ) { return x + y; }, double_vec.begin( ),
                              double_vec.end( ), result_vec.begin( ), double_vec.begin( ) );

        double zip_with_sum = std::accumulate( result_vec.begin( ), result_vec.end( ), 0 );
        REQUIRE( zip_with_sum == 400 );
    };

    SECTION( "zipWith_" ) {
        std::vector< double > double_vec( 200, 0 );
        std::fill( double_vec.begin( ), double_vec.end( ), 1 );
        std::vector< double > result_vec( 200, 0 );
        double                zip_with_sum = 0;
        clara::util::zipWith_( [&]( double x, double y ) { zip_with_sum += x + y; },
                              double_vec.begin( ), double_vec.end( ), double_vec.begin( ) );
        REQUIRE( zip_with_sum == 400 );
    };

    SECTION( "enumerate" ) {
        std::vector< double > double_vec( 200, 0 );
        for ( size_t i = 0; i < double_vec.size( ); ++i ) {
            double_vec[ i ] = static_cast< double >( i );
        }
        clara::util::enumerate( double_vec.begin( ), double_vec.end( ), 0,
                                [&]( size_t ix, double d ) { REQUIRE( ix == d ); } );
    };

    SECTION( "transform_" ) {
        std::vector< double > double_vec_A( 200, 0 );
        for ( size_t i = 0; i < double_vec_A.size( ); ++i ) {
            double_vec_A[ i ] = static_cast< double >( i );
        }

        std::vector< double > double_vec_B( 200, 0);
        size_t i = 0;
        clara::util::transform_(double_vec_B, [&](double & val)
        {
            return i++;
        });

        for ( size_t i = 0; i < double_vec_A.size( ); ++i ) {
            REQUIRE(double_vec_A[ i ] == double_vec_B[ i ]);
        }
    };
}
