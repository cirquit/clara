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
#include "../library/vehicle_state.h"
#include "../library/origin-calc.h"
#include "csv.h"

std::vector< std::tuple<object_list_t, double> > parse_csv(const std::string path)
{
    std::vector< std::tuple<object_list_t, double> > observations;
    // read the data
    io::CSVReader< 15 > in( path );
    double distance, angle, x_car, y_car, yaw_rad, v_x, v_y, color, time, timestamp, lap, a_x, a_y, steer_angle, yaw_rate;
    int    time_old      = -1;
    double timestamp_old = 0;
    while ( in.read_row( distance, angle, x_car, y_car, yaw_rad, v_x, v_y, color, time, timestamp, lap, a_x, a_y, steer_angle, yaw_rate ) ) {
        // group by timestamp
        if ( time != time_old )
        {
            object_list_t list;
            list.size = 0;
            // double t = timestamp - timestamp_old;
            timestamp_old = timestamp;
            observations.push_back( std::make_tuple(list, timestamp) );
            time_old = time;

            // std::cout // << v_x << ','
            //           // << v_y << ','
            //           // << a_x << ','
            //           // << a_y << ','
            //           // << steer_angle << ','
            //           // << yaw_rad     << ','
            //           // << yaw_rate    << ','
            //            << timestamp   << '\n';
            //              << t           << '\n';
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
        cur_object.angle_yaw     = yaw_rate;
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
    std::vector< std::tuple< object_list_t, double > > observations = parse_csv(path);
    // parametrization of data associtaion in clara
    const size_t preallocated_cluster_count           = 500;
    const size_t preallocated_detected_cones_per_step = 10;
    const double max_distance_btw_cones_m             = 2;  // meter
    const double variance_xx                          = 0.55;
    const double variance_yy                          = 0.55;
    const size_t apply_variance_step_count            = 1000000; // apply custom variance for this amount of observations
    const int    cluster_search_range                 = 10; // +/- to the min/max used cluster-index
    const int    min_driven_distance_m                = 10; // drive at least 10m until starting to check if we're near the start point
    const double lap_epsilon_m                        = 3; // if we're 0.5m near the starting point, increment the lap counter
    const double set_start_after_m                    = 5;   // we travel at least some distance until setting our start point
    std::tuple<std::string, int> log_ip_port          = std::make_tuple("0.0.0.0", 33333);
    const double max_accepted_distance_m              = 10;  // we delete every observation if it's farther than 10m
    const double origin_distance                      = 0.35; // is the distance of the COG to the camera in x

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

    clara::vehicle_state_t vs( clara::USE_KAFI_YAW );

    int counter  = 0;
    for(auto & o : observations)
    {
        if (counter++ < 1) continue;
        object_list_t & l   = std::get<0>(o);
        double                t_s = std::get<1>(o);
        if (t_s > 100) { continue; }
        if (l.size == 0) { continue; }
//        if (t_s == 0)    { continue; }
        double yaw_rate_rad = l.element[0].angle_yaw;
        double vx           = l.element[0].vx;
        double vy           = l.element[0].vy;
        double ax           = l.element[0].ax;
        double ay           = l.element[0].ay;
        double steer_angle  = l.element[0].steering_rad; 

        vs.update(vx, vy, ax, ay, 0, yaw_rate_rad, steer_angle, t_s);

        if (vx == 0) continue;

        std::cerr << "Observation [        " << counter++ << "/" << observations.size() << "]:\n"
                  << "    Rec. time:       " << vs._delta_time_s  << "s\n"
                  << "    Freq:            " << 1 / vs._delta_time_s << "Hz\n"
                  << "    velocity:        " << vs._v_x_vehicle << ", "
                                             << vs._v_y_vehicle << " m/s\n"
                  << "    yaw (mode dep.): " << vs.get_yaw()    << " rad\n"
                  << "    yaw_rate:        " << vs._yaw_rate << "rad/s\n"
                  << "    yaw_rate_steer:  " << vs._yaw_rate_steer << "rad/s\n"
                  << "    yaw_rate_kafi:   " << vs._yaw_rate_kafi << "rad/s\n";

//        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(t_s*1000)));
        std::tuple< double, double > pos;
        if (l.element[0].distance == 0)
        {
            pos = clara.add_observation( vs );  
        } else {
            origin::move_objects_by_distance(l, origin_distance);
            pos = clara.add_observation(l, vs);
        }
        std::cerr << "    pos: " << std::get<0>(pos) << "," << std::get<1>(pos) << '\n';
        std::cerr << "    lap: #" << clara.get_lap() << '\n';
        UNUSED(pos);

        if (clara.get_lap() == 1) break;

        // std::cout << yaw_rate_steer << ',' << yaw_rate_rad << '\n';

        // std::cout << l.element[0].angle_yaw << ", " << yaw << '\n';
        // std::cout << yaw_rate_rad << ',' <<  yaw_rate_steer << ',' << yaw_rate_kafi << '\n';
        // std::cout << t_s << '\n';

        std::cout << std::get<0>(pos) << ","
                   << std::get<1>(pos) << "\n";
                  //<< vs.get_yaw()     << '\n';
    }


    // python logging data
    // log_da(clara);

    return EXIT_SUCCESS;
}