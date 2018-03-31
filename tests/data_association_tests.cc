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
#include <math.h>
#include "catch.h"
#include "csv.h"

#include "../library/util.h"
#include "../library/data_association.h"

#define UNUSED(x) (void)(x)

template<size_t N>
void print_clusters(const std::array<clara::cone_state<double>, N> & clusters)
{
    // show all the cluster (modify this to maybe try to plot the results in gnuplot)
    clara::util::enumerate(clusters.begin(), clusters.end(), 0
            , [&](size_t inner_counter, const clara::cone_state<double> & cs)
    {   
        double mean_x = cs._mean_vec[0];
        double mean_y = cs._mean_vec[1];
        // don't plot unused clusters
        if (mean_x != 0 || mean_y != 0)
        {
            std::cout << "    Nr." << inner_counter << ":"
                      <<   " ~x: " << mean_x
                      <<  ", ~y: " << mean_y
                      << ", obs_count: "  << cs._observations.size()
                      << ", Covar(xx): " << cs._cov_mat[0]
                      << ", Covar(xy): " << cs._cov_mat[1]
                      << ", Covar(yy): " << cs._cov_mat[3] << '\n';
        }
    });
}


TEST_CASE("data association tests", "[data]") {

    // SECTION("example track 1 - full data test") {
    //     // maximum 250 clusters currently for each color, this is up to test the performance of vector on unbounded data
    //     const size_t N = 250;
    //     // define how many sample points are to evaluate
    //     const double MAX_SAMPLE_POINTS = 1000;
    //     // the data format our data_association works with
    //     using raw_cone_data = typename clara::data_association<N>::raw_cone_data;

    //     // read the data in exactly this format
    //     std::string csv_path = "test-data/large-map-randomized-cones-d-yw-x-y-t.csv";

    //     io::CSVReader<6> in(csv_path);
    //     // please provice exactly this header, separated by ','
    //     in.read_header(io::ignore_extra_column, "distance[m]", "angle[rad]", "x[m]", "y[m]", "color[int]", "timestamp[int]");

    //     // every inner vector is a single timestamp, starting from 0
    //     std::vector< std::vector<raw_cone_data> > yellow_cone_data;
    //     std::vector< std::vector<raw_cone_data> > blue_cone_data;
    //     std::vector< std::vector<raw_cone_data> > red_cone_data;

    //     // read the data in the vector
    //     double distance, yaw, x, y, color, timestamp;
    //     double timestamp_old = -1.0;
    //     while(in.read_row(distance, yaw, x, y, color, timestamp))
    //     {
    //         // if we already have enough sample points, stop the reading
    //         if ( timestamp >= MAX_SAMPLE_POINTS ) { break; }
    //         // will currently only use x, y and color for the classification
    //         if ( timestamp != timestamp_old )
    //         {
    //             // create new vectors for every cone color
    //             yellow_cone_data.push_back( std::vector<raw_cone_data>() );
    //             blue_cone_data.push_back( std::vector<raw_cone_data>() );
    //             red_cone_data.push_back( std::vector<raw_cone_data>() );
    //             timestamp_old = timestamp;
    //         }

    //         // watch out for this encoding, this is just for ease of use. Should change depending on the object type from darknet
    //         if( color == 0 ) yellow_cone_data.back().push_back( raw_cone_data(x, y) );
    //         if( color == 1 ) blue_cone_data.back().push_back( raw_cone_data(x, y) );
    //         if( color == 2 ) red_cone_data.back().push_back( raw_cone_data(x, y) );
    //     }

    //     // create our association object which holds all the clusters
    //     clara::data_association<N> yellow_data_association;
    //     clara::data_association<N> blue_data_association;
    //     clara::data_association<N> red_data_association;

    //     // use the data for classification
    //     for (int t = 0; t < timestamp; ++t)
    //     {
    //         // get the current timestep snapshot of visible cones
    //         auto new_yellow_cones = yellow_cone_data[t];
    //         auto new_blue_cones   = blue_cone_data[t];
    //         auto new_red_cones    = red_cone_data[t];

    //         // create or update the cluster for each color
    //         const std::array<clara::cone_state<double>, N> & current_yellow_clusters =
    //             yellow_data_association.classify_new_data(new_yellow_cones);
    //         const std::array<clara::cone_state<double>, N> & current_blue_clusters =
    //             blue_data_association.classify_new_data(new_blue_cones);
    //         const std::array<clara::cone_state<double>, N> & current_red_clusters =
    //             red_data_association.classify_new_data(new_red_cones);

    //         // print clusters
    //         std::cout << "Yellow clusters - Step #" << t << '\n';
    //         print_clusters<N>(current_yellow_clusters);
    //         std::cout << "Blue clusters - Step #" << t << '\n';
    //         print_clusters<N>(current_blue_clusters);
    //         std::cout << "Red clusters - Step #" << t << '\n';
    //         print_clusters<N>(current_red_clusters);
    //     };
    //}


    SECTION("example track 2 - partial information test") {
        // maximum 250 clusters currently for each color, this is up to test the performance of vector on unbounded data
        const size_t N = 250;
        // define how many sample points are to evaluate
        const double MAX_SAMPLE_POINTS = 10;
        // the data format our data_association works with
        using raw_cone_data = typename clara::data_association<N>::raw_cone_data;

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
            const std::vector<std::tuple<double, double>> new_cones { raw_cone_data(x, y) };
            const std::array<clara::cone_state<double>, N> & current_clusters =
                cone_data_association.classify_new_data(new_cones);

            // print clusters
            std::cout << "Clusters - Step #" << timestamp << '\n';
            print_clusters<N>(current_clusters);
        }
    }
}