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
#include "../library/clara.h"
#include "csv.h"

const std::vector< std::tuple<object_list_t, double> > parse_csv()
{
    std::vector< std::tuple<object_list_t, double> > observations;
    // read the data
    std::string csv_path = "../tests/example-data/"
                             // "round-map-10-rounds-d-a-x-y-yaw-c-t.csv";
                             "log-dist-angle-x_car-y_car-yaw_angle-vx-vy-type-time-timestamp--realtime.csv";
    io::CSVReader< 10 > in( csv_path );
    double distance, angle, x_car, y_car, yaw_rad, v_x, v_y, color, time, timestamp;
    // double distance, angle, x_car, y_car, yaw_rad, color, timestamp;
    int    time_old = -1;
    double timestamp_old = 0;
    while ( in.read_row( distance, angle, x_car, y_car, yaw_rad, v_x, v_y, color, time, timestamp ) ) {
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
        object_t & cur_object = cur_list.element[cur_list.size];
        cur_object.distance = distance;
        cur_object.angle = angle;
        cur_object.x_car = x_car;
        cur_object.y_car = y_car;
        cur_object.vx    = v_x;
        cur_object.vy    = v_y;
        cur_object.angle_yaw = yaw_rad;
        cur_object.type  = static_cast<int>(color);
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

int main(){ 
    // 
    const std::vector< std::tuple<object_list_t, double> > observations = parse_csv();
    // parametrization of data associtaion in clara
    const size_t preallocated_cluster_count           = 500;
    const size_t preallocated_detected_cones_per_step = 10;
    const double max_distance_btw_cones_m             = 2;  // meter
    const double variance_xx                          = 0.45;
    const double variance_yy                          = 0.45;
    const size_t apply_variance_step_count            = 1000; // apply custom variance for this amount of observations
    const int    cluster_search_range                 = 5; // +/- to the min/max used cluster-index
    const int    min_driven_distance_m                = 10; // drive at least 10m until starting to check if we're near the start point
    const double lap_epsilon_m                        = 0.5; // if we're 0.5m near the starting point, increment the lap counter
    const double set_start_after_m                    = 6;   // we travel at least some distance until setting our start point

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
      , std::make_tuple(0.888982, -1.50739));

    int counter  = 0;
    for(const auto & o : observations)
    {
        const object_list_t & l   = std::get<0>(o);
        const double        & t_s = std::get<1>(o);
        if (l.size == 0) { continue; }
        double yaw_rad = l.element[0].angle_yaw;
        double vx      = l.element[0].vx;
        double vy      = l.element[0].vy;

        std::cerr << "Observation [ " << counter++ << "/" << observations.size() << "]:\n"
                  << "    Rec. time: " << t_s  << "s\n"
                  << "    velocity: "  << vx << ", " << vy << " m/s\n"
                  << "    yaw: "       << yaw_rad << "rad\n";

        // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(t_s*1000)));
        std::tuple<double, double> pos = clara.add_observation(l, vx, vy, yaw_rad, t_s);
        std::cerr << "    lap: #" << clara.get_lap() << '\n';
        UNUSED(pos);
    //    if (counter > 600) { break; }
    }

    // python logging data
    log_da(clara);

    return EXIT_SUCCESS;
}
