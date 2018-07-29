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
#include "../library/clara-object.h"
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

double distance_to_zero(double x, double y)
{
    return std::sqrt(std::pow(x,2) + std::pow(y,2));
}

int main(int argc, char const *argv[]){

    if (argc != 2) { print_usage(); return EXIT_FAILURE; }
    // get the path to the csv 
    std::string path = argv[1];
    std::cerr << "[CLARA-TEST] Reading from file: \"" << path << "\"\n";

    // for error visualization
    const int               auto_scheduler_port        = 3500;
    const int               auto_scheduler_grittr_port = 3501;
    const std::string       auto_scheduler_ip   = "127.0.0.1"; // localhost

    // send clara-objects to AS
    connector::client < connector::UDP > to_autonomous_scheduler_client( auto_scheduler_port, auto_scheduler_ip);
    to_autonomous_scheduler_client.init( );
    // if we're ready, send the whole map to AS + Grittr
    connector::client < connector::UDP > to_autonomous_scheduler_grittr_client( auto_scheduler_grittr_port, auto_scheduler_ip);
    to_autonomous_scheduler_grittr_client.init( );

    // preallocation of the clara object to send to AS
    clara::object::clara_obj clara_object;

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
    const unsigned lookback_count                     = 0;       // how far do we look back for the get_clustered_observations

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
      , max_accepted_distance_m
      , lookback_count);
      //  , std::make_tuple(4.94177, 0.722539)); // hockenheim (+5m in CM)
      //, std::make_tuple(0.888982, -1.50739)); //

    // empirically estimated
    double yaw_process_noise = 0.00001;
    double bosch_variance    = 0.003;
    double steering_variance = 0.01;
    clara::vehicle_state_t vs( clara::USE_INTEGRATED_STEERING_YAW
                            ,  yaw_process_noise
                            ,  bosch_variance
                            ,  steering_variance );

    bool grittr_waiting = true;

    int counter  = 0;
    for(auto & o : observations)
    {
//        if (counter > 1000) break;
        // if (counter++ < 1) continue;
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
                  << "    yaw_rate_kafi:   " << vs._yaw_rate_kafi << "rad/s\n"
                  << "    object_list: \n";
        for (uint32_t i = 0; i < l.size; ++i)
        {
            std::cerr << "         " << l.element[i].distance << ','
                                     << l.element[i].angle    << ','
                                     << l.element[i].type    << '\n';
        }

        // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(t_s*1000)));
        if (l.element[0].distance == 0)
        {
            std::tie(clara_object.x_pos, clara_object.y_pos) = clara.add_observation( vs );  
        } else {
            origin::move_objects_by_distance(l, origin_distance);
//
//            if (clara.get_lap() >= 1)
//            {
//              if (  distance_to_zero(clara_object.x_pos, clara_object.y_pos) < 5 ){
//                  std::tie(clara_object.x_pos, clara_object.y_pos) = clara.use_observation(l, vs);
//              } else {
//                  std::tie(clara_object.x_pos, clara_object.y_pos) = clara.add_observation(l, vs);
//              }
//            } else {
                std::tie(clara_object.x_pos, clara_object.y_pos) = clara.add_observation(l, vs);
//            }
        }
        std::cerr << "    pos: " << clara_object.x_pos << "," << clara_object.y_pos << '\n';
        std::cerr << "    lap: #" << clara.get_lap() << '\n';

        if (std::isnan(clara_object.x_pos)) break;
        // for each object, use it's cluster mid-point
        clara_object.clustered_object_list = clara.get_clustered_observations(clara_object.x_pos, clara_object.y_pos, vs.get_yaw());
        // send yaw
        clara_object.yaw = vs.get_yaw();

        if (clara.get_lap() == 1 && grittr_waiting) {
            clara_object.go_grittr_flag = true;
            grittr_waiting              = false;
            // get all the yellow cones fron the data association
            auto & yellow_cluster = clara._yellow_data_association.get_cluster();
            // allocate the space needed to send the yellow cones over udp
            std::vector<clara::object::cone_position> yellow_cones;
            yellow_cones.reserve(yellow_cluster.size());
            // copy the points into the newly allocated vector
            std::for_each(yellow_cluster.begin(), yellow_cluster.end(),[&](const clara::cone_state<double> & cluster)
                    {
                    clara::object::cone_position cp;
                    cp.x_pos = cluster._mean_vec[0];
                    cp.y_pos = cluster._mean_vec[1];
                    yellow_cones.emplace_back(cp);
                    });
            // send them over udp
            size_t yellow_cones_size = yellow_cones.size(); // needed lhs
            to_autonomous_scheduler_grittr_client.send_udp<size_t>(yellow_cones_size);
            to_autonomous_scheduler_grittr_client.send_udp<clara::object::cone_position>(yellow_cones.data()[0],
                    sizeof(clara::object::cone_position) * yellow_cones.size());
            // get all the blue cones fron the data association
            auto & blue_cluster = clara._blue_data_association.get_cluster();
            // allocate the space needed to send the yellow cones over udp
            std::vector<clara::object::cone_position> blue_cones;
            blue_cones.reserve(blue_cluster.size());
            // copy the points into the newly allocated vector
            std::for_each(blue_cluster.begin(), blue_cluster.end(),[&](const clara::cone_state<double> & cluster)
                    {
                    clara::object::cone_position cp;
                    cp.x_pos = cluster._mean_vec[0];
                    cp.y_pos = cluster._mean_vec[1];
                    blue_cones.emplace_back(cp);
                    });
            // send them over udp
            size_t blue_cones_size = blue_cones.size(); // needed lhs
            to_autonomous_scheduler_grittr_client.send_udp<size_t>(blue_cones_size);
            to_autonomous_scheduler_grittr_client.send_udp<clara::object::cone_position>(blue_cones.data()[0],
                    sizeof(clara::object::cone_position) * blue_cones.size());
        } else {
            clara_object.go_grittr_flag = false;
        }
        // send clara_obj to autonomous_scheduler
        to_autonomous_scheduler_client.send_udp< clara::object::clara_obj >( clara_object );

        // std::cout << vs._yaw_rate <<  ','
        //           << vs._yaw_rate_steer << ','
        //           << vs._yaw_rate_kafi << '\n';
        std::cout << clara_object.x_pos << ","
                  << clara_object.y_pos << "\n";
        //          << vs._yaw_rate       << ","
        //          << vs.get_yaw()       << '\n';

    }


    // python logging data
    // log_da(clara);

    return EXIT_SUCCESS;
}