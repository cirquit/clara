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

#ifndef VEHICLE_STATE_H
#define VEHICLE_STATE_H

#include <tuple>
#include <vector>
#include <memory>
#include <kafi-1.0/kafi.h>
#include "memory.h"

namespace clara
{
    enum YAW_MODE : size_t { USE_NORMAL_YAW
                          ,  USE_INTEGRATED_YAW
                          ,  USE_INTEGRATED_STEERING_YAW
                          ,  USE_INTEGRATED_ACCELERATION_YAW
                          ,  USE_INTEGRATED_VEHICLE_MODEL_YAW
                          ,  USE_KAFI_YAW };

    class vehicle_state_t
    {

    // constructors
    public:
        vehicle_state_t(const YAW_MODE yaw_mode
                      , double yaw_process_noise
                      , double bosch_variance
                      , double steering_variance
                      , double acceleration_variance) 
        : _yaw_mode( yaw_mode )
        // correvit stuff
        , _v_x_vehicle( 0 )
        , _v_x_vehicle_mem( 2 )
        , _v_y_vehicle( 0 )
        , _v_y_vehicle_mem( 2 )
        // imu acceleration
        , _a_x_vehicle( 0 )
        , _a_y_vehicle( 0 )
        // recalculated correvit with specified yaw for world velocity
        , _v_x_world( 0 )
        , _v_y_world( 0 )
        // yaw from ETAS integrated from IMU
        , _yaw( 0 )
        // yaw_rate from the IMU
        , _yaw_rate( 0 )
        , _yaw_rate_mem( 2 )
        // yaw_rate from the single track model from ETAS through IMU + steering_angle @ 1000Hz
        , _yaw_rate_vm( 0 )
        , _yaw_rate_vm_mem( 2 )
        // yaw_rate calculated from acceleration (IMU)
        , _yaw_rate_acceleration( 0 )
        , _yaw_rate_acceleration_mem( 2 )
        // yaw_rate calculated from steering_angle + velocity     
        , _yaw_rate_steer( 0 )
        , _yaw_rate_steer_mem( 2 )
        // yaw_rate calculated from from all yaw_rates with a kalman filter, see _init_kafi
        , _yaw_rate_kafi( 0 )
        , _yaw_rate_kafi_mem( 2 )
        // all member where we integrated the corresponding yaw_rate
        , _integrated_yaw( 0 )
        , _integrated_acceleration_yaw( 0 )
        , _integrated_yaw_vm( 0 )
        , _integrated_steering_yaw( 0 )
        , _integrated_kafi_yaw( 0 )
        // from the steering sensor
        , _steering_angle( 0 )
        // time gone since the last call
        , _delta_time_s( 0 )
        // used for pooling yaw_rate and then nulling it after the start for each iteration
        , _yaw_rate_mean( 0 )
        , _yaw_rate_vm_mean( 0 )
        , _yaw_rate_calculated( false )
        {
            _yaw_rate_summary.reserve( 100000 );
            _yaw_rate_vm_summary.reserve( 100000 );
            _init_yaw_kafi(yaw_process_noise
                         , bosch_variance
                         , steering_variance
                         , acceleration_variance);
        }

    // methods
    public:

        /** \brief Updates all our sensors and creates new sensor data like a yaw_rate from steering_angle and v_x_vehicle
          *
          */
        void update(const double v_x_vehicle
                  , const double v_y_vehicle
                  , const double a_x_vehicle
                  , const double a_y_vehicle
                  , const double yaw
                  , const double yaw_rate
                  , const double yaw_rate_vm
                  , const double steering_angle
                  , const double delta_time_s)
        {
            // trapezoid integration (mean over two values) for all these members
            _v_x_vehicle_mem.add_value( v_x_vehicle );
            _v_y_vehicle_mem.add_value( v_y_vehicle );
            _yaw_rate_mem.add_value( yaw_rate );
            _yaw_rate_vm_mem.add_value( yaw_rate_vm );

            // set our member
            _v_x_vehicle    = _v_x_vehicle_mem.get_mean();
            _v_y_vehicle    = _v_y_vehicle_mem.get_mean();
            _a_x_vehicle    = a_x_vehicle;
            _a_y_vehicle    = a_y_vehicle;
            _yaw            = yaw;
            _yaw_rate       = _yaw_rate_mem.get_mean();
            _yaw_rate_vm    = _yaw_rate_vm_mem.get_mean();
            _steering_angle = steering_angle;
            _delta_time_s   = delta_time_s;

            // since all our original sensor members were set, we can use them to compute our missing member
            // calculate yaw rate from acceleration
            _yaw_rate_acceleration_mem.add_value( get_acceleration_yaw_rate() );
            _yaw_rate_acceleration = _yaw_rate_acceleration_mem.get_mean();
            // calculate yaw_rate from steering
            _yaw_rate_steer_mem.add_value( get_steering_yaw_rate() );
            _yaw_rate_steer = _yaw_rate_steer_mem.get_mean();
            // calculate the kafi yaw rate
            _yaw_rate_kafi_mem.add_value( get_kafi_yaw_rate() );
            _yaw_rate_kafi = _yaw_rate_kafi_mem.get_mean();
            // while we're not driving, pool the "noise"
            if (_v_x_vehicle == 0)
            {
               _yaw_rate_summary.emplace_back(_yaw_rate);
               _yaw_rate_vm_summary.emplace_back(_yaw_rate_vm);

            } else {

                if (!_yaw_rate_calculated)
                {
                    // get the mean for the normal yaw_rate
                    double yaw_rate_sum = std::accumulate(_yaw_rate_summary.begin(), _yaw_rate_summary.end(), 0);
                    _yaw_rate_mean = yaw_rate_sum / static_cast<double>(_yaw_rate_summary.size());
                    // get the mean for the vehicle model yaw_rate
                    double yaw_rate_vm_sum = std::accumulate(_yaw_rate_vm_summary.begin(), _yaw_rate_vm_summary.end(), 0);
                    _yaw_rate_vm_mean = yaw_rate_vm_sum / static_cast<double>(_yaw_rate_vm_summary.size());
                    // set the flag to subtract in the future
                    _yaw_rate_calculated = true;
                }

                // update integrated yaw while subtracting the "noise"
                _integrated_yaw += get_local_integrated_yaw( _yaw_rate_mean );
            
                // update the integrated yaw from the precalculated yaw_rate from the vehicle model in the ETAS, while subtracting the "noise"
                _integrated_yaw_vm += get_local_integrated_yaw_vm( _yaw_rate_vm_mean );

                // update integrated acceleration yaw
                _integrated_acceleration_yaw += get_local_integrated_acceleration_yaw();

                // update integrated steering yaw
                _integrated_steering_yaw += get_local_integrated_steering_yaw();
                
                // update integrated kafi yaw
                _integrated_kafi_yaw += get_local_integrated_kafi_yaw();
            }
            // translate from vehicle coordinate system to global coordiante system
            std::tie( _v_x_world, _v_y_world ) = _to_world_velocity();

        }

        //! reset all accumulated yaws
        void reset_yaws()
        {
            _integrated_yaw = 0;
            _integrated_yaw_vm = 0;
            _integrated_acceleration_yaw = 0;
            _integrated_steering_yaw = 0;
            _integrated_kafi_yaw = 0;
        }

        //! API helper function, need exactly this return type for clara
        std::tuple< double, double, double > to_world_velocity() const
        {
            return std::make_tuple(_v_x_world, _v_y_world, _delta_time_s);
        }

        //! applies the current world velocity over time to the previous postion
        std::tuple< double, double > single_shot_localization_prediction(std::tuple< double, double > old_pos) const
        {
            const double local_pos_x  = _v_x_world * _delta_time_s;
            const double local_pos_y  = _v_y_world * _delta_time_s;
            const double global_pos_x = std::get< 0 >( old_pos );
            const double global_pos_y = std::get< 1 >( old_pos );
            return std::make_tuple( local_pos_x + global_pos_x, local_pos_y + global_pos_y );
        }

        /** \brief calculates the yawrate rad/s based on the steering angle and velocity
         *    * 1.54 - Radstand
         *    * 2.73 - Übersetzung
         *    * 1.128 - Fahrzeugspur
         *    * 0.0461 - magic number from tests with a circle \todo
         */
        double get_steering_yaw_rate() const
        {
            double radius = 1.54 / (std::sin(std::abs((_steering_angle - 0.0461) / 2.73))) - 1.128 / 2;
            if (_steering_angle < 0) {
                radius = -1 * std::abs( radius );
            } else {
                radius = std::abs( radius );
            }
            double v = std::sqrt( std::pow( _v_x_vehicle, 2 ) + std::pow( _v_y_vehicle, 2 ));
            return v / radius;
        }

        //! calculate the yaw rate acceleration with the acceleration and the x velocity and y velocity
        double get_acceleration_yaw_rate() const
        {
            if( _v_x_vehicle == 0 && _v_y_vehicle == 0 ) return 0;
            return _a_y_vehicle / std::sqrt( std::pow( _v_x_vehicle, 2) + std::pow( _v_y_vehicle,2 ) );
        }

        //! calculates the local integrated yaw from the vehicle model, while subtracting the "noise"
        double get_local_integrated_yaw_vm(const double & _yaw_rate_vm_mean) const
        {
            return (_yaw_rate_vm - _yaw_rate_vm_mean) * _delta_time_s;
        }

        // get the integrated part of the local yaw calculated from the steering angle. *NOT THE YAW*, we only use the last timestep
        double get_local_integrated_steering_yaw() const
        {
            return _yaw_rate_steer * _delta_time_s;
        }

        // get the integrated part of the local yaw *NOT THE GLOBAL YAW*, we only use the last timestep, while subtracting the "noise"
        double get_local_integrated_yaw(double & _yaw_rate_mean) const
        {
            return (_yaw_rate - _yaw_rate_mean) * _delta_time_s;
        }

        // get the integrated part of the local acceleration yaw *NOT THE GLOBAL ACCELERATION YAW*
        double get_local_integrated_acceleration_yaw() const
        {
            return _yaw_rate_acceleration * _delta_time_s;
        }

        // get the integrated part of the local kafi yaw *NOT THE GLOBAL YAW*
        double get_local_integrated_kafi_yaw() const
        {
            return _yaw_rate_kafi * _delta_time_s;
        }

        //! run the kalman filter with the _yaw_rate and _yaw_rate_steer, configured by _init_yaw_kafi
        double get_kafi_yaw_rate()
        {
            // typedefs
            using mx1_vector = typename kafi::jacobian_function<N,M>::mx1_vector;
            using return_t   = typename kafi::kafi<N,M>::return_t;
            using nx1_vector = typename kafi::jacobian_function<N,M>::nx1_vector;

            // create observations
            std::shared_ptr< mx1_vector > observation = std::make_shared< mx1_vector >(
                                  mx1_vector({ { _yaw_rate              }
                                             , { _yaw_rate_steer        }
                                             , { _yaw_rate_acceleration } }));
            // update the observation
            _kafi -> set_current_observation(observation);
            // run the estimation
            return_t   result                = _kafi -> step();
            const nx1_vector estimated_state = std::get<0>(result);
            _yaw_rate_kafi = estimated_state(0,0);
            return _yaw_rate_kafi;
        }

        //! generic function to return the yaw defined in _yaw_mode in the constructor
        double get_yaw() const
        {
            switch ( _yaw_mode )
            {
                case USE_NORMAL_YAW:
                    return _yaw;
                break;
                case USE_INTEGRATED_YAW:
                    return _integrated_yaw;
                break;
                case USE_INTEGRATED_STEERING_YAW:
                    return _integrated_steering_yaw;
                case USE_INTEGRATED_ACCELERATION_YAW:
                    return _integrated_acceleration_yaw;
                break;
                case USE_INTEGRATED_VEHICLE_MODEL_YAW:
                    return _integrated_yaw_vm;
                break;
                case USE_KAFI_YAW:
                    return _integrated_kafi_yaw;
                break;
                default:
                    std::cerr << "[INFO] clara::vehicle_state_t::get_yaw(): Using a nondefined YAW_MODE " << _yaw_mode << ". Returning 0\n";
                    return 0;
                break;
            }
        }
    // methods
    private:
        //! converts the from the vehicle coordinate system (vx is always positive) to the global world coordiante system (vx may be negative), additional _delta_time_s for ease of use
        const std::tuple< double, double > _to_world_velocity()
        {
            const double v_x_world = _v_x_vehicle * std::cos( get_yaw() ) - _v_y_vehicle * std::sin( get_yaw() );
            const double v_y_world = _v_x_vehicle * std::sin( get_yaw() ) + _v_y_vehicle * std::cos( get_yaw() );
            return std::make_tuple( v_x_world, v_y_world );
        }

        //! initialize the yaw kalman filter
        void _init_yaw_kafi(double yaw_process_noise
                          , double bosch_variance
                          , double steering_variance
                          , double acceleration_variance)
        {
            // some useful typedefs
            using nx1_vector = typename kafi::jacobian_function<N,M>::nx1_vector;
            using mxm_matrix = typename kafi::jacobian_function<N,M>::mxm_matrix;
            using nxn_matrix = typename kafi::jacobian_function<N,M>::nxn_matrix;

            // state transition
            kafi::jacobian_function<N,N> f(
                std::move(kafi::util::create_identity_jacobian<N,N>()));

            // prediction scaling (state -> observations)
            kafi::jacobian_function<N,M> h(
                std::move(kafi::util::create_identity_jacobian<N,M>()));

            // given by our example, read as "the real world temperature changes are 0.1°
            nxn_matrix process_noise( { { yaw_process_noise } } );
            // given by our example, read as "both temperature sensors fluctuate by 0.8° (0.8^2 = 0.64)"
            mxm_matrix sensor_noise( { { bosch_variance, 0,                 0  }     // need to estimate the best possible noise
                                     , { 0,              steering_variance, 0  }
                                     , { 0,              0,                 acceleration_variance } }); //
            // we start with the initial state at t = 0, which we take as "ground truth", because we build the relative map around it
            nx1_vector starting_state( { { _yaw_rate } } );

            // init kalman filter
            _kafi = std::make_unique<kafi::kafi<N, M>>(std::move(f)
                                                     , std::move(h)
                                                     , starting_state
                                                     , process_noise
                                                     , sensor_noise);
        }

    // member
    public:
        //! yaw mode, determines which yaw we'll be using, predefined, integrated, integrated via steering, or kalman filtered from normal & steering
        YAW_MODE _yaw_mode;
        //! x velocity from the vehicle coordiante system (x is pointing forward, always positive)
        double _v_x_vehicle;
        //! memory for _v_x_vehicle
        memory_t _v_x_vehicle_mem;
        //! y velocity from the vehicle coordi$nte system (y is the lateral velocity)
        double _v_y_vehicle;
        //! memory for _v_y_vehicle
        memory_t _v_y_vehicle_mem;
        //! x acceleration from the vehicle coordiante system (x is pointing forward, always positive)
        double _a_x_vehicle;
        //! y acceleration from the vehicle coordiante system (y is the lateral acceleration)
        double _a_y_vehicle;
        //! x velocity of the "world", in our coordiante system of the whole map
        double _v_x_world;
        //! y velocity of the "world", in our coordiante system of the whole map
        double _v_y_world;
        //! yaw in radians of the car (may be greater than 2*PI)
        double _yaw;
        //! yaw_rate in radians/s of the car
        double _yaw_rate;
        //! memory for _yaw_rate
        memory_t _yaw_rate_mem;
        //! yaw_rate in radians/s of the car calculated from a vehicle model on the ETAS
        double _yaw_rate_vm;
        //! memory for _yaw_rate_vm
        memory_t _yaw_rate_vm_mem;
        //! acceleration yaw in m/s²
        double _yaw_acceleration;
        //! acceleration yaw rate in radians/s of the car
        double _yaw_rate_acceleration;
        //! memory for _acceleration yaw rate
        memory_t _yaw_rate_acceleration_mem;
        //! yaw_rate in radians/s of the car calculated from the steering
        double _yaw_rate_steer;
        //! memory for _yaw_rate_steer
        memory_t _yaw_rate_steer_mem;
        //! yaw_rate in radians/s of the car kalman filtered from _yaw_rate & _yaw_rate_steer
        double _yaw_rate_kafi;
        //! memory for _yaw_rate_kafi
        memory_t _yaw_rate_kafi_mem;


        //! integrated yaw_rate over time is saved here
        double _integrated_yaw;
        //! integrated yaw_rate_vm over time is saved here
        double _integrated_yaw_vm;
        //! integrated acceleration yaw rate with time is saved here
        double _integrated_acceleration_yaw;
        //! integrated steering_yaw_rate with time is saved here
        double _integrated_steering_yaw;
        //! integrated kalman filtered_yaw_rate with time is saved here \todo
        double _integrated_kafi_yaw;
        //! current steering angle position
        double _steering_angle;
        //! elapsed time in seconds since the last vehicle state
        double _delta_time_s;

        //! used to pool the yaw_rates to calculate a mean which we subtract from every future yaw_rate
        std::vector< double > _yaw_rate_summary;
        //! used to pool the yaw_rates from the vehicle model to calculate a mean which we subtract from every future yaw_rate
        std::vector< double > _yaw_rate_vm_summary;
        //! mean to subtract from every future yaw_rate
        double _yaw_rate_mean;
        //! mean to subtract from every future yaw_rate_vm
        double _yaw_rate_vm_mean;
        //! flag to start the subtraction from future yaw_rates
        bool   _yaw_rate_calculated;

        //! we estimate `kafi_yaw_rate`
        static const size_t N = 1UL;
        //! we use `yaw_rate` and `steering_yaw_rate`, `acceleration_yaw_rate`
        static const size_t M = 3UL;
        //! pointer to the velocity kalman filter which estimates the `kafi_yaw_rate` from `steering_yaw_rate` and `yaw_rate`
        std::unique_ptr<kafi::kafi<N, M>> _kafi;
    };
}


#endif // VEHICLE_STATE
