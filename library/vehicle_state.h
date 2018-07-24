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

namespace clara
{
    enum YAW_MODE : size_t { USE_NORMAL_YAW
                          ,  USE_INTEGRATED_YAW
                          ,  USE_INTEGRATED_STEERING_YAW
                          ,  USE_KAFI_YAW };

    class vehicle_state_t
    {

    // constructors
    public:
        vehicle_state_t(const YAW_MODE yaw_mode
                      , double yaw_process_noise
                      , double bosch_variance
                      , double steering_variance)
        : _yaw_mode(yaw_mode)
        , _v_x_vehicle(0)
        , _v_y_vehicle(0)
        , _a_x_vehicle(0)
        , _a_y_vehicle(0)
        , _v_x_world(0)
        , _v_y_world(0)
        , _yaw(0)
        , _yaw_rate(0)
        , _yaw_rate_steer(0)
        , _yaw_rate_kafi(0)
        , _integrated_yaw(0)
        , _integrated_steering_yaw(0)
        , _integrated_kafi_yaw(0)
        , _steering_angle(0)
        , _delta_time_s(0)
        , _yaw_rate_mean(0)
        , _yaw_rate_calculated(false)
        { 
            _yaw_rate_summary.reserve(10000);
            _init_yaw_kafi(yaw_process_noise
                         , bosch_variance
                         , steering_variance);
        }

    // methods
    public:

        /** \brief Updates all our sensors
          *
          */
        void update(const double v_x_vehicle
                  , const double v_y_vehicle
                  , const double a_x_vehicle
                  , const double a_y_vehicle
                  , const double yaw
                  , const double yaw_rate
                  , const double steering_angle
                  , const double delta_time_s)
        {
            // set all our member
            _v_x_vehicle    = v_x_vehicle;
            _v_y_vehicle    = v_y_vehicle;
            _a_x_vehicle    = a_x_vehicle;
            _a_y_vehicle    = a_y_vehicle;
            _yaw            = yaw;
            _yaw_rate       = yaw_rate;
            _steering_angle = steering_angle;
            _delta_time_s   = delta_time_s;

            if (_v_x_vehicle == 0)
            {
               _yaw_rate_summary.emplace_back(_yaw_rate);
                
            } else {
                
                if (!_yaw_rate_calculated)
                {
                    double yaw_rate_sum = std::accumulate(_yaw_rate_summary.begin(), _yaw_rate_summary.end(), 0);
                    _yaw_rate_mean = yaw_rate_sum / static_cast<double>(_yaw_rate_summary.size());
                    _yaw_rate_calculated = true;
                }

                // update integrated yaw
                _integrated_yaw += get_local_integrated_yaw() - _yaw_rate_mean;

                // calculate the yaw_rate from the steering
                _yaw_rate_steer = get_steering_yaw_rate();
                // update integrated steering yaw
                _integrated_steering_yaw += get_local_integrated_steering_yaw();
                // calculate the kafi yaw rate
                _yaw_rate_kafi = get_kafi_yaw_rate();
                // update integrated kafi yaw
                _integrated_kafi_yaw += get_local_integrated_kafi_yaw();
            }
            // translate from vehicle coordinate system to global coordiante system
            std::tie( _v_x_world, _v_y_world ) = _to_world_velocity();
            

            
        }

        // API helper function, need exactly this return type for clara 
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
         *    * 0.06 - magic number from tests with a circle \todo
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

        // get the integrated part of the local yaw calculated from the steering angle. *NOT THE YAW*, we only use the last timestep
        double get_local_integrated_steering_yaw() const
        {   
            return get_steering_yaw_rate() * _delta_time_s;
        }

        // get the integrated part of the local yaw *NOT THE GLOBAL YAW*, we only use the last timestep
        double get_local_integrated_yaw() const 
        {
            return _yaw_rate * _delta_time_s;
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
                                  mx1_vector({ { _yaw_rate   }
                                             , { _yaw_rate_steer } }));
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
                          , double steering_variance)
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
            mxm_matrix sensor_noise( { { bosch_variance, 0      }     // need to estimate the best possible noise
                                     , { 0,              steering_variance  } }); // 
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
        //! y velocity from the vehicle coordiante system (y is the lateral velocity)
        double _v_y_vehicle;
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
        //! yaw_rate in radians/s of the car calculated from the steering
        double _yaw_rate_steer;
        //! yaw_rate in radians/s of the car kalman filtered from _yaw_rate & _yaw_rate_steer
        double _yaw_rate_kafi;
        //! integrated yaw_rate with time is saved here
        double _integrated_yaw;
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
        //! mean to subtract from every future yaw_rate
        double _yaw_rate_mean;
        //! flag to start the subtraction from future yaw_rates
        bool   _yaw_rate_calculated;

        //! we estimate `kafi_yaw_rate`
        static const size_t N = 1UL;
        //! we use `yaw_rate` and `steering_yaw_rate`
        static const size_t M = 2UL;
        //! pointer to the velocity kalman filter which estimates the `kafi_yaw_rate` from `steering_yaw_rate` and `yaw_rate` 
        std::unique_ptr<kafi::kafi<N, M>> _kafi;
    };
}


#endif // VEHICLE_STATE
