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

#ifndef LAP_CNT_H
#define LAP_CNT_H

#include "clara-util.h"
#include "distance_counter.h"

/*!
 *  \addtogroup clara
 *  @{
 */

/** \brief CLARA namespace
 *
 */


namespace clara {
    //! Counts the lap, starting from 0
    class lap_counter
    {

    // constructor
    public:

        //! uses the start position without a travelled distance
        lap_counter(std::tuple<double, double> start_pos
                  , double min_driven_distance_m
                  , double lap_epsilon_m)
        : lap_counter(start_pos, min_driven_distance_m, lap_epsilon_m, 0) { }


        /** \brief Counts the laps, always checking after the min_driven_distance_m is surpassed
          * After each lap, the driven distance is resetted
          */
        lap_counter(std::tuple<double, double> start_pos
                  , double min_driven_distance_m
                  , double lap_epsilon_m
                  , double set_start_after_m)
        : _start_pos(start_pos)
        , _lap_epsilon_m(lap_epsilon_m)
        , _min_driven_distance_m(min_driven_distance_m)
        , _set_start_after_m(set_start_after_m)
        , _distance_cnt { }
        , _lap(0)
        , _unset_start(true)
        , _lap_driven_distance(0)
        { }

    // methods
    public:
        //! const distance getter
        int count() const
        {
            return _lap;
        }

        //! updates the driven distance, checks after _min_driven_distance_m is passed and checks the distance to the start point
        int add_positions(std::tuple<double, double> & old_pos
                        , std::tuple<double, double> & new_pos)
        {
            _distance_cnt.update_distance(old_pos, new_pos);
            const double & driven_distance = _distance_cnt.get_distance();

            // we set the new start position to the position after travelling at least _set_start_after_m
            if (_unset_start && _lap == 0 && driven_distance > _set_start_after_m)
            {   
                _unset_start = false;
                _start_pos = new_pos;
            }

            // after the first round, we only use the driven distance to check if we finished a lap
            if ( _lap_driven_distance != 0 &&
                 _lap_driven_distance < driven_distance )
            {
                _lap++;
                _distance_cnt.reset_distance();
                return _lap;
            } else if ( _lap_driven_distance != 0 )
            {
                return _lap;
            }

            // if we moved away enough, check for distance to start_point
            if ( !_unset_start && driven_distance >= _min_driven_distance_m )
            {
                // check the distance to our starting position
                const double to_start_distance = util::euclidean_distance(new_pos, _start_pos);
                // if we are close enough to the _start_pos, it counts as a lap
                if (to_start_distance <= _lap_epsilon_m)
                {
                    _lap++;
                    _lap_driven_distance = driven_distance;
                    _distance_cnt.reset_distance();
                }
            }
            return _lap;
        }

    // member
    private:
        std::tuple<double, double> _start_pos;
        const double               _lap_epsilon_m;
        const double               _min_driven_distance_m;
        const double               _set_start_after_m;
        distance_counter           _distance_cnt;
        int                        _lap;
        bool                       _unset_start;

        double             _lap_driven_distance;

    };
} // namespace clara

#endif // LAP_CNT_H
