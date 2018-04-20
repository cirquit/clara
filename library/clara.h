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
#include <algorithm>
#include "cone_state.h"
#include "util.h"

/*!
 *  \addtogroup clara
 *  @{
 */

/** \brief CLARA namespace
 *
 */
namespace clara {
    /** \brief
     *
     */
    class clara
    {
        //! \todo
        clara(size_t preallocated_cluster_count
            , size_t preallocated_detected_cones_per_step
            , double max_dist_btw_cones_m
            , double variance_xx
            , double variance_yy
            , size_t apply_variance_step_count)
        : yellow_data_association {
            preallocated_cluster_count, preallocated_detected_cones_per_step, max_distance_btw_cones_m,
            variance_xx, variance_yy, apply_variance_step_count }
        , blue_data_association {
            preallocated_cluster_count, preallocated_detected_cones_per_step, max_distance_btw_cones_m,
            variance_xx, variance_yy, apply_variance_step_count }
        , red_data_association {
            preallocated_cluster_count, preallocated_detected_cones_per_step, max_distance_btw_cones_m,
            variance_xx, variance_yy, apply_variance_step_count }
        }


        void add_observation(double v_x, double v_y, object_list_t * obj_list)
        {

        }




        const std::tuple< double, double > parse_object_t( object_t *obj ) {
            const double x_ = std::cos( obj->angle ) * obj->distance;
            const double y_ = std::sin( obj->angle ) * obj->distance;
            const double x = x_ * std::cos( obj->angle_yaw ) - y_ * std::sin( obj->angle_yaw );
            const double y = x_ * std::sin( obj->angle_yaw ) + y_ * std::cos( obj->angle_yaw );
        return {x + obj->x_car, y + obj->y_car};

    // member
    private:

        clara::data_association< double > yellow_data_association;
        clara::data_association< double > blue_data_association;
        clara::data_association< double > red_data_association;

}

    }
} // namespace clara

#endif // CLARA_H