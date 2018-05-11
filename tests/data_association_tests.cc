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
#include <chrono>
#include <thread>
#include <random>

#include <connector-1.0/client.h>
#include <connector-1.0/server.h>

#include "../library/data_association.h"
#include "csv.h"

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
        double cov_xx = cs._cov_mat[0];// - 0.45;
        double cov_xy = cs._cov_mat[1];
        double cov_yy = cs._cov_mat[3];// - 0.45;

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


template < class T >
void print_observations(clara::data_association<T> & da, int color)
{
    const std::vector<clara::cone_state<T>> & cluster = da.get_cluster();

    if (color == 0) { std::cout << "yellow_obs_cone_data = np.array([\n"; }
    if (color == 1) { std::cout << "blue_obs_cone_data = np.array([\n"; }
    if (color == 2) { std::cout << "red_obs_cone_data = np.array([\n"; }
    clara::util::for_each_(cluster, [&](const clara::cone_state<double> & cs)
    {
        double mean_x = cs._mean_vec[0];
        double mean_y = cs._mean_vec[1];

        if ( mean_x != 0 || mean_y != 0 ) {
            for(auto obs : cs._observations)
            {
                std::cout << "[" << obs[0] << ", " << obs[1] << "],\n"; 
            }
        }
    });
    std::cout << "]);\n";
}

const std::tuple< double, double > parse_object_t( double angle, double distance, double x_car, double y_car, double yaw_angle ) {
    const double x_ = std::cos( angle ) * distance;
    const double y_ = std::sin( angle ) * distance;

    const double x = x_ * std::cos( yaw_angle ) - y_ * std::sin( yaw_angle );
    const double y = x_ * std::sin( yaw_angle ) + y_ * std::cos( yaw_angle );
    return {x + x_car, y + y_car};
}


int main(){ 
    // define how many sample points to evaluate
    //const double MAX_SAMPLE_POINTS = 1200;
    
    // the data format, our data_association works with
    using raw_cone_data = typename clara::data_association< double >::raw_cone_data;

    // read the data
    std::string csv_path = "../tests/example-data/"
                           "wemding-map-ground-truth-cones-d-a-x-y-c-t.csv";
                          // "small-map-cm-cones-d-a-x-y-c-t.csv";
                          //   "round-map-10-rounds-d-a-x-y-yaw-c-t.csv";
    io::CSVReader< 6 > in( csv_path );
    // io::CSVReader< 7 > in( csv_path );

    std::uniform_real_distribution<double> unif(-0.5, +0.5);
    std::default_random_engine re;

    // every inner vector is a single timestamp, starting from 0
    std::vector< std::vector< raw_cone_data > > yellow_cone_data;
    std::vector< std::vector< raw_cone_data > > blue_cone_data;
    std::vector< std::vector< raw_cone_data > > red_cone_data;

    // connector::client< connector::UDP > to_visualizing_client( 33333, "10.158.78.232" ); // , "127.0.0.1" );
    // to_visualizing_client.init( );

    // read the data in the vector
    double distance, angle, x, y, color, timestamp;
    // double distance, angle, x, y, yaw_angle, color, timestamp;
    int    timestamp_old = -1;
    
    while ( in.read_row( distance, angle, x, y, color, timestamp ) ) {
    // while ( in.read_row( distance, angle, x, y, yaw_angle, color, timestamp ) ) {
       
        // group by timestamp
        if ( timestamp != timestamp_old )
        {
            yellow_cone_data.push_back( std::vector< raw_cone_data >( ) );
            blue_cone_data.push_back( std::vector< raw_cone_data >( ) );
            red_cone_data.push_back( std::vector< raw_cone_data >( ) );
            timestamp_old = timestamp;
        }
        // x = unif(re) + x;
        // y = unif(re) + y;
        raw_cone_data raw_cone = { x , y, 0, 0};
        // raw_cone_data raw_cone = parse_object_t(angle, distance, x, y, yaw_angle);

        // watch out for this encoding, this is just for ease of use. Should
        // change depending on the object type from darknet
        if ( color == 0 )
            yellow_cone_data.back( ).push_back( raw_cone );
        if ( color == 1 )
            blue_cone_data.back( ).push_back( raw_cone );
        if ( color == 2 )
            red_cone_data.back( ).push_back( raw_cone );
    }

    // parametrization of data associtaion
    const size_t preallocated_cluster_count           = 500;
    const size_t preallocated_detected_cones_per_step = 10;
    const double max_distance_btw_cones_m             = 2;  // meter
    const double variance_xx                          = 0.45;
    const double variance_yy                          = 0.45;
    const size_t apply_variance_step_count            = 100;
    const int    cluster_search_range                 = 5; // +/- to the min/max used cluster-index

    // data association for each color
    clara::data_association< double > yellow_data_association(
        preallocated_cluster_count, preallocated_detected_cones_per_step, max_distance_btw_cones_m,
        variance_xx, variance_yy, apply_variance_step_count, cluster_search_range );
    clara::data_association< double > blue_data_association(
        preallocated_cluster_count, preallocated_detected_cones_per_step, max_distance_btw_cones_m,
        variance_xx, variance_yy, apply_variance_step_count, cluster_search_range );
    clara::data_association< double > red_data_association(
        preallocated_cluster_count, preallocated_detected_cones_per_step, max_distance_btw_cones_m,
        variance_xx, variance_yy, apply_variance_step_count, cluster_search_range );
    // do the association
    clara::util::zipWith_(
        [&]( const std::vector< raw_cone_data > &cur_yellow_cones,
             const std::vector< raw_cone_data > &cur_blue_cones,
             const std::vector< raw_cone_data > &cur_red_cones ) {

                UNUSED(cur_red_cones);
                auto & y_cluster = yellow_data_association.classify_new_data( cur_yellow_cones );
                auto & b_cluster = blue_data_association.classify_new_data( cur_blue_cones );
                // red_data_association.classify_new_data( cur_red_cones );
                auto & y_cluster_index = yellow_data_association.get_detected_cluster_ixs();
                auto & b_cluster_index = blue_data_association.get_detected_cluster_ixs();

                UNUSED(y_cluster);
                UNUSED(y_cluster_index);
                UNUSED(b_cluster);
                UNUSED(b_cluster_index);
                // for( auto observed : cur_yellow_cones )
                // {
                //     std::cerr << std::get<0>(observed) << "," << std::get<1>(observed) << ';';
                // }
                // for( auto observed : cur_blue_cones )
                // {
                //     std::cerr << std::get<0>(observed) << "," << std::get<1>(observed) << ';';
                // }
                // std::cerr << "|";
                // for( auto cluster_ix : y_cluster_index )
                // {
                //     auto cluster = y_cluster[cluster_ix];
                //     std::cerr << cluster._mean_vec[0] << "," << cluster._mean_vec[1] << ","
                //               << cluster._cov_mat[0]  << "," << cluster._cov_mat[1]  << "," << cluster._cov_mat[3] << ","
                //               << cluster_ix           << "," << "0;";
                // }
                // for( auto cluster_ix : b_cluster_index )
                // {
                //     auto cluster = b_cluster[cluster_ix];
                //     std::cerr << cluster._mean_vec[0] << "," << cluster._mean_vec[1] << ","
                //               << cluster._cov_mat[0]  << "," << cluster._cov_mat[1]  << "," << cluster._cov_mat[3] << ","
                //               << cluster_ix           << "," << "1;";
                // }
                // // no localization yet
                // std::cerr << "|0,0|\n";


                // std::cerr << "Counter: " << counter++ << '\n';
                // std::this_thread::sleep_for(std::chrono::milliseconds(5));
                // if(counter++ % 50 == 0){
                //     for(auto & c : y_cluster)
                //     {   
                //         std::string stuff = std::to_string(c._mean_vec[0]) + ", " + std::to_string(c._mean_vec[1]) + ", 0";
                //         std::cerr << "Sending - " << stuff << '\n';
                //         std::string size = std::to_string(stuff.size());

                //         to_python_client.send_tcp<const char>(size.c_str()[0], sizeof(char) * size.size());
                //         to_python_client.send_tcp<const char>(stuff.c_str()[0], sizeof(char) * stuff.size());
                     
                //         std::this_thread::sleep_for(std::chrono::milliseconds(10));
                //     }
                //     for(auto & c : b_cluster)
                //     {   
                //         std::string stuff = std::to_string(c._mean_vec[0]) + ", " + std::to_string(c._mean_vec[1]) + ", 1";
                //         std::cerr << "Sending - " << stuff << '\n';
                //         std::string size = std::to_string(stuff.size());

                //         to_python_client.send_tcp<const char>(size.c_str()[0], sizeof(char) * size.size());
                //         to_python_client.send_tcp<const char>(stuff.c_str()[0], sizeof(char) * stuff.size());
                     
                //         std::this_thread::sleep_for(std::chrono::milliseconds(10));
                //     }
                // }
        },
        yellow_cone_data.begin( ), yellow_cone_data.end( ), blue_cone_data.begin( ),
        red_cone_data.begin( )
    );

    // print out the python file
    std::cout << "import numpy as np\n";
    print_data_assoc< double >(yellow_data_association, 0);
    print_data_assoc< double >(blue_data_association, 1);
    print_data_assoc< double >(red_data_association, 2);
    print_observations< double >(yellow_data_association, 0);
    print_observations< double >(blue_data_association, 1);
    print_observations< double >(red_data_association, 2);


//    }
}

