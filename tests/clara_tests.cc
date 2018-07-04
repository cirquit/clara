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
#include <kafi-1.0/kafi.h>

#include "../library/data_association.h"
#include "../library/clara.h"
#include "csv.h"


//! calculates the yawrate rad/s based on the steering angle and velocity
double calc_yawrate_from_steering(const double & steer_angle
                                , const double & vx
                                , const double & vy)
{
    // 1.54 - Radstand
    // 2.73 - Übersetzung
    // 1.128 - Fahrzeugspur
    double radius =  1.54 / (std::sin(std::abs(steer_angle / 2.73))) - 1.128 / 2;

    if (steer_angle < 0)
    {
        radius =  -1 * std::abs(radius);
    } else {
        radius = std::abs(radius);
    }
    double v = std::sqrt(std::pow(vx,2) + std::pow(vy,2));
    double yaw_rate = v / radius;
    
    return yaw_rate;
}

//! get yaw rate (very noisy and bad practice, use yaw_rate from can)
double get_yaw_rate(const object_t & obj
                  , const double & old_yaw
                  , const double & time_s )
{
    // std::cerr << "yaw:    " << obj.angle_yaw << '\n'
    //           << "oldyaw: " << old_yaw    << '\n'
    //           << "delta_t:" << time_s     << '\n';
    return (obj.angle_yaw - old_yaw) / time_s;
}

const std::vector< std::tuple<object_list_t, double> > parse_csv(const std::string path)
{
    std::vector< std::tuple<object_list_t, double> > observations;
    // read the data
    io::CSVReader< 14 > in( path );
    double distance, angle, x_car, y_car, yaw_rad, v_x, v_y, color, time, timestamp, lap, a_x, a_y, steer_angle;
    // double distance, angle, x_car, y_car, yaw_rad, color, timestamp;
    int    time_old      = -1;
    double timestamp_old = 0;
    while ( in.read_row( distance, angle, x_car, y_car, yaw_rad, v_x, v_y, color, time, timestamp, lap, a_x, a_y, steer_angle ) ) {
        // group by timestamp
        if ( time != time_old )
        {
            object_list_t list;
            list.size = 0;
            double t = timestamp - timestamp_old;
            timestamp_old = timestamp;
            observations.push_back( std::make_tuple(list, t) );
            time_old = time;
        }

        object_list_t & cur_list = std::get<0>(observations.back());
        object_t & cur_object    = cur_list.element[cur_list.size];
        cur_object.distance      = distance;
        cur_object.angle         = angle;
        cur_object.x_car         = x_car;
        cur_object.y_car         = y_car;
        cur_object.vx            = v_x;
        cur_object.vy            = v_y;
        cur_object.ax            = a_x;
        cur_object.ay            = a_y;
        cur_object.angle_yaw     = yaw_rad;
        cur_object.type          = static_cast<int>(color);
        cur_object.steering_rad  = steer_angle;
        cur_object.time_s        = timestamp;
        cur_list.size++;
    }
    return observations;
}

void log_da(clara::clara & clara)
{

   clara::data_association<double> & yellow_data_association = clara._yellow_data_association;
   clara::data_association<double> & blue_data_association = clara._blue_data_association;
   clara::data_association<double> & red_data_association = clara._red_data_association; 
   // print out the python file
   std::cout << "import numpy as np\n";
   yellow_data_association.print_data_assoc(0);
   blue_data_association.print_data_assoc(1);
   red_data_association.print_data_assoc(2);
   yellow_data_association.print_observations(0);
   blue_data_association.print_observations(1);
   red_data_association.print_observations(2);
}

void print_usage()
{
    std::cerr << "Usage: clara_test <path-to-csv>\n";
}

int main(int argc, char const *argv[]){

    if (argc != 2) { print_usage(); return EXIT_FAILURE; }
    // get the path to the csv 
    std::string path = argv[1];
    std::cerr << "[CLARA-TEST] Reading from file: \"" << path << "\"\n";
    // 
    const std::vector< std::tuple<object_list_t, double> > observations = parse_csv(path);
    // parametrization of data associtaion in clara
    const size_t preallocated_cluster_count           = 500;
    const size_t preallocated_detected_cones_per_step = 10;
    const double max_distance_btw_cones_m             = 2.5;  // meter
    const double variance_xx                          = 0.45;
    const double variance_yy                          = 0.45;
    const size_t apply_variance_step_count            = 100000; // apply custom variance for this amount of observations
    const int    cluster_search_range                 = 5; // +/- to the min/max used cluster-index
    const int    min_driven_distance_m                = 10; // drive at least 10m until starting to check if we're near the start point
    const double lap_epsilon_m                        = 1.5; // if we're 0.5m near the starting point, increment the lap counter
    const double set_start_after_m                    = 0;   // we travel at least some distance until setting our start point
    const double max_accepted_distance_m              = 10;  // we delete every observation if it's farther than 10m
    std::tuple<std::string, int> log_ip_port          = std::make_tuple("0.0.0.0", 33333);

    clara::clara clara(
        preallocated_cluster_count
      , preallocated_detected_cones_per_step
      , max_distance_btw_cones_m
      , variance_xx
      , variance_yy
      , apply_variance_step_count
      , cluster_search_range
      , min_driven_distance_m
      , lap_epsilon_m
      , set_start_after_m
      , log_ip_port
      , max_accepted_distance_m); 
      //  , std::make_tuple(4.94177, 0.722539)); // hockenheim (+5m in CM)
      //, std::make_tuple(0.888982, -1.50739)); //

    // set the starting state the same as the first yaw (we may null this in the future)
    double yaw_rate_steer = 0; // get_yaw_rate(std::get<0>(*(observations.begin() + 1)).element[0], 0, std::get<1>(*(observations.begin() + 1)));
    int counter  = 0;

    // dirty kafi
    const size_t N = 1UL;
    const size_t M = 2UL;

    using mx1_vector = typename kafi::jacobian_function<N,M>::mx1_vector;
    using nx1_vector = typename kafi::jacobian_function<N,M>::nx1_vector;
    using mxm_matrix = typename kafi::jacobian_function<N,M>::mxm_matrix;
    using nxn_matrix = typename kafi::jacobian_function<N,M>::nxn_matrix;
    using return_t   = typename kafi::kafi<N,M>::return_t;

    // state transition
    kafi::jacobian_function<N,N> f(
        std::move(kafi::util::create_identity_jacobian<N,N>()));

    // prediction scaling (state -> observations)
    kafi::jacobian_function<N,M> h(
        std::move(kafi::util::create_identity_jacobian<N,M>()));

    // given by our example, read as "the real world temperature changes are 0.1°
    nxn_matrix process_noise( { { 0.0001 } } );
    // given by our example, read as "both temperature sensors fluctuate by 0.8° (0.8^2 = 0.64)"
    mxm_matrix sensor_noise( { { 0.005, 0      }     // need to estimate the best possible noise
                             , { 0,     0.003  } }); // 
  // we start with the initial state at t = 0, which we take as "ground truth", because we build the relative map around it
    nx1_vector starting_state( { { yaw_rate_steer } } );

    // init kalman filter
    kafi::kafi<N,M> kafi(std::move(f)
                       , std::move(h)
                       , starting_state
                       , process_noise
                       , sensor_noise);
    double yaw_rate_kafi = yaw_rate_steer;
    double yaw     = std::get<0>(*(observations.begin() + 1)).element[0].angle_yaw;
    double old_yaw = yaw;
    for(const auto & o : observations)
    {
        if (counter++ < 1) continue;
        const object_list_t & l   = std::get<0>(o);
        const double        & t_s = std::get<1>(o);
        if (t_s > 100) { continue; }
        
        if (l.size == 0) { continue; }
        if (t_s == 0)    { continue; }
        double yaw_rate_rad = get_yaw_rate(l.element[0], old_yaw, t_s);
        double vx           = l.element[0].vx;
        double vy           = l.element[0].vy;
        double ax           = l.element[0].ax;
        double ay           = l.element[0].ay;
        double steer_angle  = l.element[0].steering_rad; 

        double yaw_rate_steer = calc_yawrate_from_steering(steer_angle, vx, vy);
        // if (yaw_steer < 0)
        // {
        //     yaw_steer = 2*3.1415926 - yaw_steer;
        // }

        // we start with the same yaw for both measurements
        std::shared_ptr< mx1_vector > observation = std::make_shared< mx1_vector >(
                              mx1_vector({ { yaw_rate_rad   }
                                         , { yaw_rate_steer } }));

        if (yaw_rate_steer != yaw_rate_rad) {
            // update the observation
            kafi.set_current_observation(observation);
            // run the estimation
            return_t   result          = kafi.step();
            const nx1_vector estimated_state = std::get<0>(result);
            yaw_rate_kafi = estimated_state(0,0);
        }

        std::cerr << "Observation [ " << counter++ << "/" << observations.size() << "]:\n"
                  << "    Rec. time: " << t_s  << "s\n"
                  << "    velocity: "  << vx << ", " << vy << " m/s\n"
                  << "    yaw_rate: "       << yaw_rate_rad << "rad/s\n"
                  << "    yaw_rate_steer: " << yaw_rate_steer << "rad/s\n"
                  << "    yaw_rate_kafi: "  << yaw_rate_kafi << "rad/s\n";
        yaw += (yaw_rate_rad + 0.0272834) * t_s;
        old_yaw = yaw;

        double inserted_yaw = yaw_rate_kafi * t_s;
        // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(t_s*1000)));
        std::tuple<double, double> pos = clara.add_observation(l, vx, vy, inserted_yaw, ax, ay, steer_angle, t_s);
        std::cerr << "    pos: " << std::get<0>(pos) << "," << std::get<1>(pos) << '\n';
        std::cerr << "    lap: #" << clara.get_lap() << '\n';
        UNUSED(pos);

        // std::cout << yaw_rate_steer << ',' << yaw_rate_rad << '\n';

        // std::cout << l.element[0].angle_yaw << ", " << yaw << '\n';
        std::cout << yaw_rate_rad << ',' <<  yaw_rate_steer << ',' << yaw_rate_kafi << '\n';
        // std::cout << std::get<0>(pos) << "," << std::get<1>(pos) << '\n';
    }


    // python logging data
    //log_da(clara);

    return EXIT_SUCCESS;
}
