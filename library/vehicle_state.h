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
        vehicle_state_t(const YAW_MODE yaw_mode)
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
        { }

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
            // update integrated yaw
            _integrated_yaw += get_local_integrated_yaw();
            //! calculate the yaw_rate from the steering
            _yaw_rate_steer = get_steering_yaw_rate();
            // update integrated steering yaw
            _integrated_steering_yaw += get_local_integrated_steering_yaw();
            // update integrated kalman filtered yaw
            // \todo
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
         */
        double get_steering_yaw_rate() const
        {
            double radius = 1.54 / (std::sin(std::abs(_steering_angle / 2.73))) - 1.128 / 2;
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

        // get the integrated part of the local yaw *NOT THE YAW*, we only use the last timestep
        double get_local_integrated_yaw() const 
        {
            return _yaw_rate * _delta_time_s;
        }

        //! generic function to return the yaw defined in _yaw_mode in the constructor
        double get_yaw() const 
        {
            switch ( _yaw_mode )
            {
                case USE_NORMAL_YAW:
                    std::cerr << "[INFO] clara::vehicle_state_t::get_yaw(): using normal yaw: " << _yaw << '\n';
                    return _yaw;
                break;
                case USE_INTEGRATED_YAW:
                    std::cerr << "[INFO] clara::vehicle_state_t::get_yaw(): using integrated yaw: " << _integrated_yaw << '\n';
                    return _integrated_yaw;
                break;
                case USE_INTEGRATED_STEERING_YAW:
                    return _integrated_steering_yaw;
                break;
                case USE_KAFI_YAW:
                    std::cerr << "[INFO] clara::vehicle_state_t::get_yaw(): Using a USE_KAFI_YAW is not implemented yet\n";
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
    };
}


#endif // VEHICLE_STATE
