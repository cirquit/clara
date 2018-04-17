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
#include <iostream>
#include <math.h>
#include <vector>
#include <cstdlib>
#include <random>
#include <iomanip>
#include <string>

#include "../library/csv.h"
#include "../library/data_association.h"
#include "../library/util.h"

template < size_t N >
void print_clusters( const std::array< clara::cone_state< double >, N > &clusters, int color ) {
    UNUSED(color);
    // show all the cluster (modify this to maybe try to plot the results in
    // gnuplot)
    clara::util::enumerate( clusters.begin( ), clusters.end( ), 0,
                            [&]( size_t inner_counter, const clara::cone_state< double > &cs ) {
                                UNUSED(inner_counter);

                                double mean_x = cs._mean_vec[ 0 ];
                                double mean_y = cs._mean_vec[ 1 ];
                                // don't plot unused clusters
                                if ( mean_x != 0 || mean_y != 0 ) {
                                    std::cout << mean_x << ", " << mean_y << "\n";
                                    //        std::cout << color << ", " << inner_counter << ", " <<
                                    //        mean_x << ", " << mean_y
                                    //                           << ", " << cs._cov_mat[0]
                                    //                           << ", " << cs._cov_mat[1]
                                    //                           << ", " << cs._cov_mat[3] << '\n';
                                    
                                        std::cout << "    Nr." << inner_counter << ":"
                                                  <<   " ~x: " << mean_x
                                                  <<  ", ~y: " << mean_y
                                                  << ", obs_count: "  << cs._observations.size()
                                                  << ", Covar(xx): " << cs._cov_mat[0]
                                                  << ", Covar(xy): " << cs._cov_mat[1]
                                                  << ", Covar(yy): " << cs._cov_mat[3] << '\n';
                                                  
                                }
                            } );
}

// template< size_t N >
// void print_data_assoc(clara::data_association<N> & da, int color)
// {   
//     const std::array<clara::cone_state<double>, N> & cluster = da.get_current_cluster();
//   //  const std::array<double, N> & weights = da.get_cone_state_weights();

//     if (color == 0) { std::cout << "yellow_cone_data = np.array([\n"; }
//     if (color == 1) { std::cout << "blue_cone_data = np.array([\n"; }
//     if (color == 2) { std::cout << "red_cone_data = np.array([\n"; }
//     clara::util::zipWith_([&](const clara::cone_state<double> & cs) //, double cluster_weight)
//     {
//         double mean_x = cs._mean_vec[0];
//         double mean_y = cs._mean_vec[1];
//         double cov_xx = cs._cov_mat[0];
//         double cov_yy = cs._cov_mat[3];
//         if (cs._observations.size() < 4)
//         {
//             cov_xx -= 0.25;
//             cov_yy -= 0.25;
//         }
//         double cov_xy = cs._cov_mat[1];

//         std::string np_b = "np.array([";
//         std::string np_e = "])";
//         if ( mean_x != 0 || mean_y != 0 ) {

//             std::cout << np_b
//                       << "[" << mean_x << ", " << mean_y << "]"  << ", "
//                       << "[[" << cov_xx << ", " << cov_xy << "]" << ", "
//                       << "[" << cov_xy << ", " << cov_yy << "]]" << ", "
//                      // << "["  << cluster_weight << "]"
//                       << np_e << ",\n"; 
//         }
//     }, cluster.begin(), cluster.end()); //, weights.begin());

//     std::cout << "]);\n";
// }

template< size_t N >
void print_ground_truth(std::vector< std::vector< typename clara::data_association< N >::raw_cone_data> > & cones, std::string id)
{

    std::cout << '\n' << id << "_cone_ground_truth = np.array([";

    for(auto cur_timestamp : cones)
    {
        for(auto cone : cur_timestamp)
        {
            std::cout << '[' << std::get<0>(cone) << ", " << std::get<1>(cone) << "],\n";
        }
    }

    std::cout << "]);\n";
}

int main(int argc, char *argv[])
{   
    UNUSED(argc);
    UNUSED(argv);

    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> dist(-1, 1);

    // maximum 250 clusters currently for each color, this is up to test the
    // performance of vector on unbounded data
    const size_t N = 250;
    // define how many sample points to evaluate
    const double MAX_SAMPLE_POINTS = 1200;
    // the data format, our data_association works with
    using raw_cone_data = typename clara::data_association< N >::raw_cone_data;

    // read the data in exactly this format
    std::string csv_path = "../tests/example-data/"
                           "wemding-map-ground-truth-cones-d-a-x-y-c-t.csv";

    io::CSVReader< 6 > in( csv_path );
    // please provide exactly this header, separated by ','
    //      in.read_header(io::ignore_extra_column, "distance[m]",
    //      "angle[rad]", "x[m]", "y[m]", "color[int]", "timestamp[int]");

    // every inner vector is a single timestamp, starting from 0
    std::vector< std::vector< raw_cone_data > > yellow_cone_data;
    std::vector< std::vector< raw_cone_data > > blue_cone_data;
    std::vector< std::vector< raw_cone_data > > red_cone_data;

    // read the data in the vector
    double distance, angle, x, y, color, timestamp;
    int    timestamp_old = -1;
    
    while ( in.read_row( distance, angle, x, y, color, timestamp ) ) {
        // if we already have enough sample points, stop the reading
        if ( timestamp >= MAX_SAMPLE_POINTS ) {
            break;
        }
        // will currently only use x, y and color for the classification
        if ( timestamp != timestamp_old ) {
            // create new vectors for every cone color
            yellow_cone_data.push_back( std::vector< raw_cone_data >( ) );
            blue_cone_data.push_back( std::vector< raw_cone_data >( ) );
            red_cone_data.push_back( std::vector< raw_cone_data >( ) );
            timestamp_old = timestamp;
        }

        // watch out for this encoding, this is just for ease of use. Should
        // change depending on the object type from darknet

        double x_noisy = x + dist(e2);
        double y_noisy = y + dist(e2);

        if ( color == 0 ) // yellow
            yellow_cone_data.back( ).push_back( raw_cone_data(x_noisy, y_noisy ) );
        if ( color == 1 ) // blue
              blue_cone_data.back( ).push_back( raw_cone_data(x_noisy, y_noisy ) );
        if ( color == 2 ) // red
              red_cone_data.back( ).push_back( raw_cone_data(x_noisy, y_noisy ) );
    }

    // create our association object which holds all the clusters
    clara::data_association< N > yellow_data_association;
    clara::data_association< N > blue_data_association;
    clara::data_association< N > red_data_association;
//    int i = 0;
    clara::util::zipWith_(
        [&]( const std::vector< raw_cone_data > &cur_yellow_cones,
             const std::vector< raw_cone_data > &cur_blue_cones,
             const std::vector< raw_cone_data > &cur_red_cones ) {
            // create or update the cluster for each color

            const std::array< clara::cone_state< double >, N > &current_yellow_clusters =
                yellow_data_association.classify_new_data( cur_yellow_cones );
            const std::array< clara::cone_state< double >, N > &current_blue_clusters =
                blue_data_association.classify_new_data( cur_blue_cones );
            const std::array< clara::cone_state< double >, N > &current_red_clusters =
                red_data_association.classify_new_data( cur_red_cones );
        },
        yellow_cone_data.begin( ), yellow_cone_data.end( ), blue_cone_data.begin( ),
        red_cone_data.begin( )
    );

    std::cout << "import numpy as np\n";
    // print_data_assoc< N >(yellow_data_association, 0);
    // print_data_assoc< N >(blue_data_association, 1);
    // print_data_assoc< N >(red_data_association, 2);

    print_ground_truth< N >(yellow_cone_data, "yellow");
    print_ground_truth< N >(blue_cone_data, "blue");
    print_ground_truth< N >(red_cone_data, "red");

    return EXIT_SUCCESS;
}
