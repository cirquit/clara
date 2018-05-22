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

#ifndef CLARA_H
#define CLARA_H

#include <functional>
#include <iostream>
#include <tuple>
#include <memory>
#include <algorithm>
#include <kafi-1.0/kafi.h>
#include <connector-1.0/client.h>

#include "../external/object-list/include/object.h"
#include "cone_state.h"
#include "data_association.h"
#include "clara-util.h"
#include "lap_counter.h"
#include "maybe.h"
#include "search_cones.h"

/*!
 *  \addtogroup clara
 *  @{
 */

/** \brief CLARA namespace
 *
 */
namespace clara {
    /** \brief
     * Main fusion point between Data Association and Localization. Wrapper for everything
     */
    class clara
    {

    // typedefs
    public:
        //! noisy `x`, `y` position and relative `x`, `y` position rotated by the respective yaw at the time (normalized) 
        using raw_cone_data = std::tuple<double, double, double, double>;
        //! a cone position (x,y)
        using cone_position = std::array<double, 2>;
        //! 4 cones with positional information
        using near_cones    = std::array<cone_position, 2>;
        //! maybe returns 4 cones, if we have enough objects to create it
        using maybe_cones = typename concept::maybe<std::tuple<near_cones, near_cones>>;
    // constructor
    public:
        //! contructor with default starting position at 0,0
        clara(size_t preallocated_cluster_count
            , size_t preallocated_detected_cones_per_step
            , double max_dist_btw_cones_m
            , double variance_xx
            , double variance_yy
            , size_t apply_variance_step_count
            , int cluster_search_range
            , int min_driven_distance_m
            , double lap_epsilon_m
            , double set_start_after_m
            , std::tuple<std::string, int> log_ip_port)
        : clara(preallocated_cluster_count
            , preallocated_detected_cones_per_step
            , max_dist_btw_cones_m
            , variance_xx
            , variance_yy
            , apply_variance_step_count
            , cluster_search_range
            , min_driven_distance_m
            , lap_epsilon_m
            , set_start_after_m
            , log_ip_port
            , std::make_tuple(0.0, 0.0)) { }

        //! constructor
        clara(size_t preallocated_cluster_count
            , size_t preallocated_detected_cones_per_step
            , double max_dist_btw_cones_m
            , double variance_xx
            , double variance_yy
            , size_t apply_variance_step_count
            , int cluster_search_range
            , int min_driven_distance_m
            , double lap_epsilon_m
            , double set_start_after_m
            , std::tuple<std::string, int> log_ip_port
            , std::tuple<double, double> starting_position) 
        : _yellow_data_association {
            preallocated_cluster_count, preallocated_detected_cones_per_step, max_dist_btw_cones_m,
            variance_xx, variance_yy, apply_variance_step_count, cluster_search_range }
        , _blue_data_association {
            preallocated_cluster_count, preallocated_detected_cones_per_step, max_dist_btw_cones_m,
            variance_xx, variance_yy, apply_variance_step_count, cluster_search_range }
        , _red_data_association {
            preallocated_cluster_count, preallocated_detected_cones_per_step, max_dist_btw_cones_m,
            variance_xx, variance_yy, apply_variance_step_count, cluster_search_range }
        , _lap_counter(starting_position, min_driven_distance_m, lap_epsilon_m, set_start_after_m)
        , _log_client(std::get<1>(log_ip_port), std::get<0>(log_ip_port))
        { 
            // initialize the logging server
            _log_client.init();
            // observation preallocation
            _new_yellow_cones.reserve(preallocated_detected_cones_per_step);
            _new_blue_cones.reserve(preallocated_detected_cones_per_step);
            _new_red_cones.reserve(preallocated_detected_cones_per_step);
            // used cluster index preallocation to compare in the next step
            _yellow_detected_cluster_ixs_old.reserve(preallocated_detected_cones_per_step);
            _blue_detected_cluster_ixs_old.reserve(preallocated_detected_cones_per_step);
            _red_detected_cluster_ixs_old.reserve(preallocated_detected_cones_per_step);
            // used vector to hold all currently calculated cone velocities (2* because of both yellow and blue)
            _velocities.reserve(2 * preallocated_detected_cones_per_step);
            // we start at the zero position
            _estimated_position = starting_position;
            _estimated_position_old = starting_position;
            // init kafi
            _init_kafi();
            // init clock for velocity to distance calculation
            _start_clock();
        }

        //! main function - parse obj_list and use all external sensor data to do the data association and localization
        const std::tuple<double, double> & add_observation(const object_list_t & obj_list
                                                         , double v_x_sensor
                                                         , double v_y_sensor
                                                         , double yaw_rad
                                                         , double timestep_s = 0)
        {
            // prepare preallocated raw cone lists for new cones
            _new_yellow_cones.clear();
            _new_blue_cones.clear();
            _new_red_cones.clear();
            // std::cerr << "    [clara.h::add_observation()]\n";
            // calculate difference from last call to start_clock(), if it's zero, we have to calculate it by ourselves
            if (timestep_s == 0){
                // std::cerr << "        measuring time: ";
                timestep_s = _get_diff_time_s();
                // std::cerr << timestep_s << "s\n";
            } else 
            {
                // std::cerr << "        got time: " << timestep_s << "s\n";
            }
            // iterate over the c-style array and apped cones based on their color
            for(uint32_t i = 0; i < obj_list.size; ++i)
            {
                if (obj_list.element[i].type == 0) _new_yellow_cones.emplace_back(_parse_object_t(obj_list.element[i], v_x_sensor, v_y_sensor, timestep_s, yaw_rad));
                if (obj_list.element[i].type == 1) _new_blue_cones.emplace_back(  _parse_object_t(obj_list.element[i], v_x_sensor, v_y_sensor, timestep_s, yaw_rad));
                if (obj_list.element[i].type == 2) _new_red_cones.emplace_back(   _parse_object_t(obj_list.element[i], v_x_sensor, v_y_sensor, timestep_s, yaw_rad));
            }
            // erase cones by maximum allowed distance
            _erase_by_distance(_new_yellow_cones, v_x_sensor, v_y_sensor, timestep_s);
            _erase_by_distance(_new_blue_cones, v_x_sensor, v_y_sensor, timestep_s);
            _erase_by_distance(_new_red_cones, v_x_sensor, v_y_sensor, timestep_s);
            // sort by relative distance to our current position, so the mapping step has some ordering and can do sanity checks
            _sort_by_rel_distance(_new_yellow_cones, v_x_sensor, v_y_sensor, timestep_s);
            _sort_by_rel_distance(_new_blue_cones, v_x_sensor, v_y_sensor, timestep_s);
            _sort_by_rel_distance(_new_red_cones, v_x_sensor, v_y_sensor, timestep_s);
            // do the data association (\todo parallelize me with openmp tasks)
            _yellow_data_association.classify_new_data(_new_yellow_cones);
            _blue_data_association.classify_new_data(_new_blue_cones);
            _red_data_association.classify_new_data(_new_red_cones);
            // estimate the velocity based on the detected cones (saved in (color)_detected_cluster_ixs_old)
            const std::tuple<double, double, double> velocity_t = std::make_tuple(v_x_sensor, v_y_sensor, timestep_s);
            // const std::tuple<double, double, double> velocity_t = _estimate_velocity(v_x_sensor, v_y_sensor, timestep_s);
            // update the position based on the estimated v_x, v_y and the time
            const std::tuple<double, double> new_position = _apply_physics_model(velocity_t);
            _update_estimated_position(new_position);
            // update the travelled distance to check if we are close enough to the start
            _lap_counter.add_positions(_estimated_position_old, _estimated_position);
            std::cerr << "        calc velocity: " << std::get<0>(velocity_t) << "m/s, " << std::get<1>(velocity_t) << "m/s\n" \
                      << "        real velocity: " << obj_list.element[0].vx << "m/s, " << obj_list.element[0].vy << "m/s\n" \
                      << "        prev position: " << std::get<0>(_estimated_position_old) << ", " << std::get<1>(_estimated_position_old) << '\n' \
                      << "        new  position: " << std::get<0>(new_position) << ", " << std::get<1>(new_position) << '\n' \
                      << "        real position: " << obj_list.element[0].x_car << ", " << obj_list.element[0].y_car << '\n';
            _log_visualization_udp(obj_list.element[0].x_car, obj_list.element[0].y_car, yaw_rad);
            // _log_position(obj_list.element[0].x_car, obj_list.element[0].y_car, std::get<0>(new_position), std::get<1>(new_position));
            // 
            return _estimated_position;
        }

        //! return the current amount of laps
        int get_lap() const {
            return _lap_counter.count();
        }

        //! returns the cone infront of us and then behind us, respectivly yellow and blue, can fail if we don't have enough observations
        maybe_cones get_basecase_cones()
        {
            return util::get_basecase_cones(_estimated_position, _yellow_data_association, _blue_data_association);
        }

    // methods
    private:
        //! csv position logging function
        void _log_position(const double & ground_truth_x_car
                         , const double & ground_truth_y_car
                         , const double & estimated_x_car
                         , const double & estimated_y_car) 
        {
            std::cout << ground_truth_x_car << ","
                      << ground_truth_y_car << ","
                      << estimated_x_car    << ","
                      << estimated_y_car    << '\n';
        }

        //! create a string from the logs and send them to the udp logger server \todo make parallel
        void _log_visualization_udp(const double & x_pos
                                  , const double & y_pos
                                  , const double & yaw_rad)
        {
            std::ostringstream os;
            _log_visualization(x_pos, y_pos, yaw_rad, os);
            std::string msg(os.str());
            _log_client.send_udp<const char>(msg.c_str()[0], msg.size());
        }

        //! log the clara visualization to cout
        void _log_visualization_cout(const double & x_pos
                                   , const double & y_pos
                                   , const double & yaw_rad)
        {
            _log_visualization(x_pos, y_pos, yaw_rad, std::cout);
        }

        //! logging in the predefined scheme to a stream object
        void _log_visualization(const double & x_pos
                              , const double & y_pos
                              , const double & yaw_rad
                              , std::ostream & stream)
        {
            stream << "CLARA|";
            for(auto & y : _new_yellow_cones)
            {
                stream << std::get<0>(y) << "," << std::get<1>(y) << ";";
            }
            for(auto & b : _new_blue_cones)
            {
                stream << std::get<0>(b) << "," << std::get<1>(b) << ";";
            }
            stream << "|";
            auto & yellow_cones       = _yellow_data_association.get_cluster();
            auto & yellow_cluster_ixs = _yellow_data_association.get_detected_cluster_ixs();
            for(auto & y_ix : yellow_cluster_ixs)
            {
                const auto & yc = yellow_cones[y_ix];
                stream << yc._mean_vec[0] << "," \
                          << yc._mean_vec[1] << "," \
                          << yc._cov_mat[0]  << "," \
                          << yc._cov_mat[1]  << "," \
                          << yc._cov_mat[3]  << "," \
                          << "y" << y_ix     << "," \
                          << 0               << ";";
            }
            auto & blue_cones       = _blue_data_association.get_cluster();
            auto & blue_cluster_ixs = _blue_data_association.get_detected_cluster_ixs();
            for(auto & b_ix : blue_cluster_ixs)
            {
                const auto & bc = blue_cones[b_ix];
                stream << bc._mean_vec[0] << "," \
                          << bc._mean_vec[1] << "," \
                          << bc._cov_mat[0]  << "," \
                          << bc._cov_mat[1]  << "," \
                          << bc._cov_mat[3]  << "," \
                          << "b" << b_ix     << "," \
                          << 1               << ";";
            }
            stream << "|";
            stream << x_pos << ","
                      << y_pos << ","
                      << yaw_rad << '\n';
        }

        //! how to parse an object_t to get from the relative distance and angle position to the absolute localization
        const std::tuple< double, double, double, double > _parse_object_t( const object_t & obj
                                                                          , const double & v_x_sensor
                                                                          , const double & v_y_sensor
                                                                          , const double & timestep_s
                                                                          , const double & yaw_rad ) const
        { 
            // apply local velocity to the previously estimated position
            double x_car, y_car;
            std::tie(x_car, y_car) = _predict_position_single_shot(v_x_sensor, v_y_sensor, timestep_s);

            // trigonometry - get x,y position from angle and distance
            const double x_ = std::cos( obj.angle ) * obj.distance;
            const double y_ = std::sin( obj.angle ) * obj.distance;
            // rotate the x,y coordiantes with the yaw of the car (rotation around y in car model)
            const double x = x_ * std::cos( yaw_rad ) - y_ * std::sin( yaw_rad );
            const double y = x_ * std::sin( yaw_rad ) + y_ * std::cos( yaw_rad );
            // std::cerr << "    [clara.h:parse_object_t()]\n"
            //           << "        x_car_old: " << x_car_old << ", y_car_old: " << y_car_old << '\n'
            //           << "            x_car: " << x_car     << ",     y_car: " << y_car << '\n'
            //           << "                x: " << x         << ",         y: " << y     << '\n'
            //           << "            x_abs: " << x + x_car << ",     y_abs: " << y + y_car << '\n';
            return std::make_tuple(x + x_car, y + y_car, x, y);
        }

        //! returns the estimated velocity_t (x,y, timestep_s)
        const std::tuple<double, double, double> _estimate_velocity(double v_x_sensor
                                                                  , double v_y_sensor
                                                                  , double timestep_s)
        {
            std::cerr << "    [clara.h:estimate_velocity()]\n";
            using nx1_vector = typename kafi::jacobian_function<N,M>::nx1_vector;
            using mx1_vector = typename kafi::jacobian_function<N,M>::mx1_vector;
            using return_t   = typename kafi::kafi<N,M>::return_t;
            // get cluster by reference to not change ownership
            auto & yellow_cluster = _yellow_data_association.get_cluster();
            auto & blue_cluster   = _blue_data_association.get_cluster();
            // get currently seen cluster indexes by reference
            const auto & yellow_detected_cluster_ix = _yellow_data_association.get_detected_cluster_ixs();
            const auto & blue_detected_cluster_ix   = _blue_data_association.get_detected_cluster_ixs();
            // delete previous velocities
            _velocities.clear();
            // for each newly detected yellow cluster
            for(const size_t & y_ix : yellow_detected_cluster_ix)
            {
                // if we saw it in the previous step
                if (util::contains(_yellow_detected_cluster_ixs_old, y_ix))
                {
                    auto y_velocity = _calculate_velocity(y_ix, yellow_cluster, timestep_s);
                    _velocities.emplace_back( y_velocity );
                }
            }
            // for each newly detected blue cluster
            for(const size_t & b_ix : blue_detected_cluster_ix)
            {
                // if we saw it in the previous step
                if (util::contains(_blue_detected_cluster_ixs_old, b_ix))
                {
                    auto b_velocity = _calculate_velocity(b_ix, blue_cluster, timestep_s);
                    _velocities.emplace_back( b_velocity );
                }
            }
            // update the newly seen cluster indicies
            _update_detected_cluster(yellow_detected_cluster_ix, blue_detected_cluster_ix);
            // if we don't see any new cones, we can't update the kalman filter
            if (_velocities.empty())
            {   
                std::cerr << "        - no matched cluster observations, returning velocities from correvit\n";
                return std::make_tuple(v_x_sensor, v_y_sensor, timestep_s);
            }
            // calculate mean velocity changes for every cone
            const std::tuple<double, double> cone_velocites = _get_mean_velocities(_velocities);
            const double v_x_cones = std::get<0>(cone_velocites);
            const double v_y_cones = std::get<1>(cone_velocites);
            // for debugging purposes
            if (true) // v_x_sensor == 0 && v_y_sensor == 0)
            {   
                std::cerr << "        - vx/vy_sensor are zero, returning velocities from cones\n";
                return std::make_tuple(v_x_cones, v_y_cones, timestep_s);
            }
            // run the kalman filter with sensor and cone velocities
            std::shared_ptr< mx1_vector > observation = std::make_shared< mx1_vector >(
                            mx1_vector( { { v_x_sensor }, { v_y_sensor }
                                        , { v_x_cones }, { v_y_cones } } ));
            std::cerr << "       - observation: " << v_x_sensor << '\n'
                      << "                      " << v_y_sensor << '\n'
                      << "                      " << v_x_cones << '\n'
                      << "                      " << v_y_cones << '\n';
            (*_kafi).set_current_observation(observation);
            const return_t   result          = (*_kafi).step();
            const nx1_vector estimated_state = std::get<0>(result);
            const double estimated_v_x = estimated_state(0, 0);
            const double estimated_v_y = estimated_state(1, 0);
            //
            return std::make_tuple(estimated_v_x, estimated_v_y, timestep_s);
        }

        //! unsafe function, should only be called if we can access cone._observations[last/last-1] and cluster[ix]
        const std::tuple<double, double> _calculate_velocity(const size_t ix
                                                           , const std::vector<cone_state<double>> & cluster
                                                           , const double timestep_s) const
        {   
            const cone_state<double> & cone = cluster[ix];
            double distance_x = 0;
            double distance_y = 0;
            std::tie(distance_x, distance_y) = cone.get_relative_pos_difference();

            const double v_x = distance_x / timestep_s;
            const double v_y = distance_y / timestep_s;

            std::cerr << "    [clara.h:calculate_velocity()]\n"
                      << "        ix: " << ix << ", time: " << timestep_s << "s\n" \
                      << "        distance_x: " << distance_x << "m\n" \
                      << "        distance_y: " << distance_y << "m\n" \
                      << "            vx: " << v_x << " m/s\n" \
                      << "            vy: " << v_y << " m/s\n";
            return std::make_tuple(v_x, v_y); 
        }

        //! replaces _yellow_detected_cluster_ixs, _blue_detected_cluster_ixs and with the new detected cluster indices 
        void _update_detected_cluster(const std::vector<size_t> & yellow_detected_cluster_ixs,
                                      const std::vector<size_t> & blue_detected_cluster_ixs)
        {
            _yellow_detected_cluster_ixs_old.clear();
            _blue_detected_cluster_ixs_old.clear();
            std::copy(yellow_detected_cluster_ixs.begin(), yellow_detected_cluster_ixs.end(), std::back_inserter(_yellow_detected_cluster_ixs_old));
            std::copy(blue_detected_cluster_ixs.begin(), blue_detected_cluster_ixs.end(), std::back_inserter(_blue_detected_cluster_ixs_old));
        }

        //! handy naming
        void _update_estimated_position(const std::tuple<double, double> & new_position)
        {
            _estimated_position_old = _estimated_position;
            _estimated_position     = new_position;
        }

        //! inizialize the velocity kalman filter
        void _init_kafi()
        {
            // typedefs
            using mx1_vector = typename kafi::jacobian_function<N,M>::mx1_vector;
            using nx1_vector = typename kafi::jacobian_function<N,M>::nx1_vector;
            using mxm_matrix = typename kafi::jacobian_function<N,M>::mxm_matrix;
            using nxn_matrix = typename kafi::jacobian_function<N,M>::nxn_matrix;
            using h_func          = std::function<void(nx1_vector &, mx1_vector &)>;
            using par_jacobi_func = std::function<double(const nx1_vector &)>;
            using h_jacobi_func   = kafi::jacobian_function<N,M>::jacobi_func;
            // state transition does nothing (we don't know)
            kafi::jacobian_function<N,N> f(
                std::move(kafi::util::create_identity_jacobian<N,N>()));
            // propagte the estimated v_x and v_y to both sensors
            const h_func _h = [](nx1_vector & in, mx1_vector & out)
            {
                out(0,0) = in(0,0); // estimated v_x
                out(1,0) = in(1,0); // estimated v_y
                out(2,0) = in(0,0); // estimated v_x
                out(3,0) = in(1,0); // estimated v_y
            };
            // simple constant functions
            const par_jacobi_func dh_one  = kafi::util::identity_derivative<N>(1);
            const par_jacobi_func dh_zero = kafi::util::identity_derivative<N>(0);
            const h_jacobi_func _H
            {
                { dh_one,  dh_zero }
             ,  { dh_zero, dh_one  }
             ,  { dh_one,  dh_zero }
             ,  { dh_zero, dh_one  }
            };
            // state to observation mapping (state -> observations)
            kafi::jacobian_function<N,M> h(_h, _H);
            // read as "the real world velocity fluctuates between 0.1m/s" (0.1**2 = 0.01)
            nxn_matrix process_noise( { { 0.01, 0    }
                                      , { 0,    0.01 } } );
            /* read as
             *
             * * v_x / v_y from the correvit have a noise of +/-0.5% of 1.25km/h (datasheet)-> 0.347m/s -> (^2) -> 0.121
             * * v_x / v_y from the cones have a noise of (\todo - let's try a bigger offset ~ 0.5m/s -> (^2) 0.25)
             */ 
            mxm_matrix sensor_noise( { { 0.121,     0,    0,     0 }
                                     , { 0,     0.121,    0,     0 } 
                                     , { 0,         0, 0.25,     0 }
                                     , { 0,         0,    0,  0.25 } });
            // we start at velocity 0 in both x and y
            nx1_vector starting_state(0);
            // init kalman filter
            _kafi = std::make_unique<kafi::kafi<N, M>>(std::move(f)
                                                     , std::move(h)
                                                     , starting_state
                                                     , process_noise
                                                     , sensor_noise);
        }

        /** \brief calculate the mean velocities
          * INVARIANT: velocities is nonempty
          */
        const std::tuple<double, double> _get_mean_velocities(const std::vector<std::tuple<double, double>> & velocities) const
        {
            return util::mean_accumulate(velocities);
        }

        //! starts the clock to use with get_diff_time_s
        void _start_clock()
        {
            _start = std::chrono::high_resolution_clock::now();
        }

        //! get time difference from last call to start_clock in seconds. Restarts the clock
        double _get_diff_time_s()
        {
            auto end = std::chrono::high_resolution_clock::now();
            int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - _start).count();
            _start_clock();
            return elapsed_ms / 1000.0;
        }

        //! calculates the new position based on the estimated v_x, v_y and the elapsed time
        const std::tuple<double, double> _apply_physics_model(const std::tuple<double, double, double> & velocity_t) const
        {
            const double v_x        = std::get<0>(velocity_t);
            const double v_y        = std::get<1>(velocity_t);
            const double timestep_s = std::get<2>(velocity_t);

            const double pos_x = (v_x * timestep_s) + std::get<0>(_estimated_position);
            const double pos_y = (v_y * timestep_s) + std::get<1>(_estimated_position);

            return std::make_tuple(pos_x, pos_y);
        }

        //! apply local velocity if we are too slow
        std::tuple<double, double> _predict_position_single_shot(const double & v_x_sensor
                                                               , const double & v_y_sensor
                                                               , const double & timestep_s) const
        {
            const double x_car_old = std::get<0>(_estimated_position); // obj.x_car; // update with v_x_sensor
            const double y_car_old = std::get<1>(_estimated_position); // obj.y_car; // update with v_y_sensor
            const double x_car = x_car_old + (v_x_sensor * timestep_s);
            const double y_car = y_car_old + (v_y_sensor * timestep_s); 
            return std::make_tuple(x_car, y_car);
        }

        //! sort by relative distance to our currently best estimated position
        void _sort_by_rel_distance(std::vector<raw_cone_data> & cones
                                 , const double & v_x_sensor
                                 , const double & v_y_sensor
                                 , const double & timestep_s)
        {
            std::tuple<double, double> car_pos = _predict_position_single_shot(v_x_sensor, v_y_sensor, timestep_s);
            std::sort(cones.begin(), cones.end(), [&](raw_cone_data & a, raw_cone_data & b)
            {
                const std::tuple<double, double> _a = std::make_tuple( std::get<0>(a), std::get<1>(a) );
                const std::tuple<double, double> _b = std::make_tuple( std::get<0>(b), std::get<1>(b) );
                return util::euclidean_distance(_a, car_pos) < util::euclidean_distance(_b, car_pos);
            });
        }

        //! erase cones by distance
        void _erase_by_distance(std::vector<raw_cone_data> & cones
                              , const double & v_x_sensor
                              , const double & v_y_sensor
                              , const double & timestep_s)
        {
            std::tuple<double, double> car_pos = _predict_position_single_shot(v_x_sensor, v_y_sensor, timestep_s);
            cones.erase(std::remove_if(cones.begin(), 
                                       cones.end(),
                [&](raw_cone_data c)
                {
                    const std::tuple<double, double> _a = std::make_tuple( std::get<0>(c), std::get<1>(c) );
                    return util::euclidean_distance(_a, car_pos) > 10;
                }),
            cones.end());
        }

    // member
    public:
        //! data association 
        data_association< double > _yellow_data_association;
        //! data association 
        data_association< double > _blue_data_association;
        //! data association 
        data_association< double > _red_data_association;

        //! temporary storage for the observations
        std::vector<raw_cone_data> _new_yellow_cones;
        //! temporary storage for the observations
        std::vector<raw_cone_data> _new_blue_cones;
        //! temporary storage for the observations
        std::vector<raw_cone_data> _new_red_cones;

        //! temporary storage for last classified cluster index
        std::vector<size_t> _yellow_detected_cluster_ixs_old;
        std::vector<size_t> _blue_detected_cluster_ixs_old;
        std::vector<size_t> _red_detected_cluster_ixs_old;

        //! holds the caluculated velocities of the current observation 
        std::vector<std::tuple<double, double>> _velocities;

        //! estimated world position
        std::tuple<double, double> _estimated_position;
        //! estimated world position from the last timestep
        std::tuple<double, double> _estimated_position_old;
        //! we estimate `v_x` and `v_y` with kafi (2x1)
        static const size_t N = 2;
        //! we observe `v_x` and `v_y` from cones and from the correvit (2x2)
        static const size_t M = 4;
        //! pointer to the velocity kalman filter which estimates the velocity in `x` and `y` from the correvit and cone velocity
        std::unique_ptr<kafi::kafi<N, M>> _kafi;
        //! clock to apply velocities from last observed model
        std::chrono::time_point<std::chrono::high_resolution_clock> _start;
        //! counts the lap based on the travelled distance and the set start_position 
        lap_counter _lap_counter;

        //! log client which accepts the clara logs
        connector::client< connector::UDP > _log_client;
    };
} // namespace clara

#endif // CLARA_H
