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

#ifndef DISTANCE_CNT_H
#define DISTANCE_CNT_H

#include "util.h"

/*!
 *  \addtogroup clara
 *  @{
 */

/** \brief CLARA namespace
 *
 */
namespace clara {
    //! Accumulates the distance based on the difference of two positions, used for lap counting in clara.h
    class distance_counter
    {

    // constructor
    public:
        //! default constructor, starting with a zero distance
        distance_counter()
        : _distance_m(0) { }

    // methods
    public:
        //! const distance getter
        double get_distance() const
        {
            return _distance_m;
        }

        //! euclidean distance of two positions appended to _distance_m
        void update_distance(const std::tuple<double, double> & old_pos
                           , const std::tuple<double, double> & new_pos)
        {
            _distance_m += util::euclidean_distance(old_pos, new_pos);
        }

        void reset_distance()
        {
            _distance_m = 0;
        }

    // member
    private:
        //! driven distance in meter
        double _distance_m;
    };
} // namespace clara

#endif // DISTANCE_CNT_H