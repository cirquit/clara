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

#include "util.h"
#include "distance_counter.h"

/*!
 *  \addtogroup clara
 *  @{
 */

/** \brief CLARA namespace
 *
 */
namespace clara {
    /// Counts the lap, starting from 0
    class lap_counter
    {

    // constructor
    public:
        /** \brief Counts the laps, always checking after the min_driven_distance_m is surpassed
          * After each lap, the driven distance is resetted
          */
        lap_counter(std::tuple<double, double> start_pos
                  , double min_driven_distance_m
                  , double lap_epsilon_m)
        : _distance_cnt { }
        , _min_driven_distance_m(min_driven_distance_m)
        , _lap_epsilon_m(lap_epsilon_m)
        , _start_pos(start_pos)
        , _laps(0)
        { }

    // methods
    public:
        //! const distance getter
        int count() const
        {
            return _laps;
        }

        //! sets the starting position, if we need to update it
        void set_start_pos(const std::tuple<double, double> start_pos)
        {
            _start_pos = start_pos;
        }

        //! updates the driven distance, checks after _min_driven_distance_m is passed and checks the distance to the start point
        int add_positions(std::tuple<double, double> & old_pos
                              , std::tuple<double, double> & new_pos)
        {
            _distance_cnt.update_distance(old_pos, new_pos);
            const double & driven_distance = _distance_cnt.get_distance();
            // if we moved away enough, check for distance to start_point
            if ( driven_distance >= _min_driven_distance_m)
            {
                // check the distance to our starting position
                const double to_start_distance = util::euclidean_distance(new_pos, _start_pos);
                // if we are close enough to the _start_pos, it counts as a lap
                if (to_start_distance <= _lap_epsilon_m)
                {
                    _laps++;
                    _distance_cnt.reset_distance();
                }
            }
            return _laps;
        }

    // member
    private:
        std::tuple<double, double> _start_pos;
        const double               _lap_epsilon_m;
        const double               _min_driven_distance_m;
        distance_counter           _distance_cnt;
        int                        _laps;
    };
} // namespace clara

#endif // LAP_CNT_H