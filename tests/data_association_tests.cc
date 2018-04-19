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
#include "csv.h"
#include <blaze/Math.h>
#include <iostream>
#include <math.h>
#include <vector>

#include "../library/data_association.h"
#include "../library/util.h"

template< class T >
void print_data_assoc(clara::data_association<T> & da, int color)
{   
    const std::vector<clara::cone_state<T>> & cluster = da.get_cluster();
    const double cluster_weight = 0; // deprecated, numpy needs this value to have a valid matrix \todo fix python script

    if (color == 0) { std::cout << "yellow_cone_data = np.array([\n"; }
    if (color == 1) { std::cout << "blue_cone_data = np.array([\n"; }
    if (color == 2) { std::cout << "red_cone_data = np.array([\n"; }
    clara::util::for_each_(cluster, [&](const clara::cone_state<double> & cs)
    {
        double mean_x = cs._mean_vec[0];
        double mean_y = cs._mean_vec[1];
        double cov_xx = cs._cov_mat[0];
        double cov_xy = cs._cov_mat[1];
        double cov_yy = cs._cov_mat[3];

        std::string np_b = "np.array([";
        std::string np_e = "])";
        if ( mean_x != 0 || mean_y != 0 ) {
            std::cout << np_b
                      << "[" << mean_x << ", " << mean_y << "]"  << ", "
                      << "[[" << cov_xx << ", " << cov_xy << "]" << ", "
                      << "[" << cov_xy << ", " << cov_yy << "]]" << ", "
                      << "["  << cluster_weight << "]"
                      << np_e << ",\n"; 
        }
    });

    std::cout << "]);\n";
}



TEST_CASE( "data association tests", "[data]" ) {

    SECTION( "example track 1 - full data test" ) {
        // define how many sample points to evaluate
        const double MAX_SAMPLE_POINTS = 1200;
        // the data format, our data_association works with
        using raw_cone_data = typename clara::data_association< double >::raw_cone_data;

        // read the data
        std::string csv_path = "../tests/example-data/"
                               "wemding-map-ground-truth-cones-d-a-x-y-c-t.csv";
        io::CSVReader< 6 > in( csv_path );

        // every inner vector is a single timestamp, starting from 0
        std::vector< std::vector< raw_cone_data > > yellow_cone_data;
        std::vector< std::vector< raw_cone_data > > blue_cone_data;
        std::vector< std::vector< raw_cone_data > > red_cone_data;

        // read the data in the vector
        double distance, angle, x, y, color, timestamp;
        int    timestamp_old = -1;
        
        while ( in.read_row( distance, angle, x, y, color, timestamp ) ) {
            // if we already have enough sample points, stop the reading
            if ( timestamp >= MAX_SAMPLE_POINTS ) { break; }
            // group by timestamp
            if ( timestamp != timestamp_old )
            {
                yellow_cone_data.push_back( std::vector< raw_cone_data >( ) );
                blue_cone_data.push_back( std::vector< raw_cone_data >( ) );
                red_cone_data.push_back( std::vector< raw_cone_data >( ) );
                timestamp_old = timestamp;
            }
            // watch out for this encoding, this is just for ease of use. Should
            // change depending on the object type from darknet
            if ( color == 0 )
                yellow_cone_data.back( ).push_back( raw_cone_data( x, y ) );
            if ( color == 1 )
                blue_cone_data.back( ).push_back( raw_cone_data( x, y ) );
            if ( color == 2 )
                red_cone_data.back( ).push_back( raw_cone_data( x, y ) );
        }

        // parametrization of data associtaion
        const size_t preallocated_cluster_count           = 500;
        const size_t preallocated_detected_cones_per_step = 10;
        const double max_distance_btw_cones_m             = 2;  // meter
        const double variance_xx                          = 0.25;
        const double variance_yy                          = 0.25;
        const size_t apply_variance_step_count            = 4;

        // data association for each color
        clara::data_association< double > yellow_data_association(
            preallocated_cluster_count, preallocated_detected_cones_per_step, max_distance_btw_cones_m,
            variance_xx, variance_yy, apply_variance_step_count );
        clara::data_association< double > blue_data_association(
            preallocated_cluster_count, preallocated_detected_cones_per_step, max_distance_btw_cones_m,
            variance_xx, variance_yy, apply_variance_step_count );
        clara::data_association< double > red_data_association(
            preallocated_cluster_count, preallocated_detected_cones_per_step, max_distance_btw_cones_m,
            variance_xx, variance_yy, apply_variance_step_count );

        // do the association
        clara::util::zipWith_(
            [&]( const std::vector< raw_cone_data > &cur_yellow_cones,
                 const std::vector< raw_cone_data > &cur_blue_cones,
                 const std::vector< raw_cone_data > &cur_red_cones ) {
                
                    yellow_data_association.classify_new_data( cur_yellow_cones );
                    blue_data_association.classify_new_data( cur_blue_cones );
                    red_data_association.classify_new_data( cur_red_cones );
            },
            yellow_cone_data.begin( ), yellow_cone_data.end( ), blue_cone_data.begin( ),
            red_cone_data.begin( )
        );

        // print out the python file
        std::cout << "import numpy as np\n";
        print_data_assoc< double >(yellow_data_association, 0);
        print_data_assoc< double >(blue_data_association, 1);
        print_data_assoc< double >(red_data_association, 2);
    }
}
/*
    SECTION("example track 2 - partial information test") {
        // maximum 250 clusters currently for each color, this is up to test the
   performance of vector on unbounded data
        const size_t N = 250;
        // define how many sample points are to evaluate
        const double MAX_SAMPLE_POINTS = 10;
        // the data format our data_association works with
        using raw_cone_data = typename
   clara::data_association<N>::raw_cone_data;

        // read data from file
        std::string csv_path = "test-data/large-map-randomized-cones-xy.csv";
        io::CSVReader<2> in(csv_path);

        // init the data association pipeline
        clara::data_association<N> cone_data_association;

        // read the data in the vector
        double x, y, timestamp;
        timestamp = 0;
        while(in.read_row(x, y))
        {
            // if we already have enough sample points, stop the reading
            if ( timestamp >= MAX_SAMPLE_POINTS ) { break; }
            timestamp++;

            // construct a single observation, because we don't have timestamps
            const std::vector<std::tuple<double, double>> new_cones {
   raw_cone_data(x, y) };
            const std::array<clara::cone_state<double>, N> & current_clusters =
                cone_data_association.classify_new_data(new_cones);

            // print clusters
            std::cout << "Clusters - Step #" << timestamp << '\n';
            print_clusters<N>(current_clusters);
        }
    }
    */
