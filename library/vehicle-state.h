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
    class vehicle_state_t
    {

    public:

        /** \brief
          *
          */
        void update(const double v_x_vehicle
                  , const double v_y_vehicle
                  , const double yaw_rad
                  , const double timestep_s)
        {
            // set all our member
            _v_x_vehicle = v_x_vehicle;
            _v_y_vehicle = v_y_vehicle;
            _yaw_rad     = yaw_rad;
            _timestep_s  = timestep_s;
            // translate from vehicle coordinate system to global coordiante system
            std::tie(_v_x_world, _v_y_world) = _to_world_velocity(_v_x_vehicle, _v_y_vehicle, _yaw_rad);
            // compute the local x and y coordinates
            std::tie(_local_pos_x, _local_pos_y) = _single_shot_localization_prediction()
        }

        //! converts the from the vehicle coordinate system (vx is always positive) to the global world coordiante system (vx may be negative)
        const std::tuple< double, double > _to_world_velocity() const
        {
            const double v_x_world = _v_x_sensor * std::cos( _yaw_rad ) - _v_y_sensor * std::sin( _yaw_rad );
            const double v_y_world = _v_x_sensor * std::sin( _yaw_rad ) + _v_y_sensor * std::cos( _yaw_rad );
            return std::make_tuple(v_x_world, v_y_world, timestep_s);
        }

        //! applies the current world velocity over time
        const std::tuple< double, double > _single_shot_localization_prediction() const
        {
            const double local_pos_x = _v_x_world / _timestep_s;
            const double local_pos_y = _v_y_world / _timestep_s;
            return std::make_tuple(local_pos_x, local_pos_y);
        }

    // member
    public:

        //! x velocity from the vehicle coordiante system (x is pointing forward, always positive)
        double _v_x_vehicle;
        //! y velocity from the vehicle coordiante system (y is the lateral velocity)
        double _v_y_vehicle;
        //! x velocity of the "world", in our coordiante system of the whole map
        double _v_x_world;
        //! y velocity of the "world", in our coordiante system of the whole map
        double _v_y_world;
        //! yaw in radians of the car (may be greater than 2*PI)
        double _yaw_rad;
        //! elapsed time in seconds since the last vehicle state
        double _timestep_s;
        //! local position based on the current _v_x_world and _timestep_s
        double _local_pos_x;
        //! local position based on the current _v_y_world and _timestep_s
        double _local_pos_y;
    };
}


#endif // VEHICLE_STATE