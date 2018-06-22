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

#ifndef REFINED_CONES_H
#define REFINED_CONES_H

#include "../external/object-list/include/object.h"
#include "cone_state.h"


namespace clara {

    namespace util {

        //! coord class with x and y, only used in refined_cones.h
        struct coord
        {
            double x;
            double y;
        };

        //! rotate (x,y) by angle phi (radian)
        struct coord rotation(const struct coord c, const double phi)
        {
            double x_ = c.x * std::cos( phi ) - c.y * std::sin( phi ); // rotated x position
            double y_ = c.x * std::sin( phi ) + c.y * std::cos( phi ); // rotated y position
            struct coord c_;
            c_.x = x_;
            c_.y = y_;
            return c_;
        }

        //! converts a clustered cone into an object_t, this includes the calculation with rotation from global coords
        object_t to_object_t(const cone_state<double> & cs
                           , const double x_pos
                           , const double y_pos
                           , const double yaw)
        {
            const struct coord shifted_coord = { cs._mean_vec[0] - x_pos, cs._mean_vec[1] - y_pos };
            const struct coord rotated_coord = rotation(shifted_coord ,-yaw);
            double distance = std::sqrt(std::pow(rotated_coord.x, 2) + std::pow(rotated_coord.y, 2));
            double angle = acos( rotated_coord.x / (distance * 1) ); 

            object_t obj;
            obj.distance = distance;
            obj.angle    = angle;
            return obj;
        }

        //! appends the object to the object_list, increments the size
        void append_object(const object_t object, object_list_t & object_list)
        {
//            std::cout << "object.type: "     << object.type << '\n';
//            std::cout << "object.distance: " << object.distance << '\n';
//            std::cout << "object_list.size: " << object_list.size << '\n';
            object_list.element[ object_list.size ] = object;
            object_list.size++;
        }

        //! appends the blue cone to the object_list by converting it into the object 
        void append_blue_cone(const cone_state<double> & cs
                            , const double x_pos
                            , const double y_pos
                            , const double yaw
                            , object_list_t & object_list)
        {   
            object_t object = to_object_t(cs, x_pos, y_pos, yaw);
            object.type = 1;
            append_object(object, object_list);
        }
        //! appends the yellow cone to the object_list by converting it into the object 
        void append_yellow_cone(const cone_state<double> & cs
                              , const double x_pos
                              , const double y_pos
                              , const double yaw
                              , object_list_t & object_list)
        {   
            object_t object = to_object_t(cs, x_pos, y_pos, yaw);
            object.type = 0;
            append_object(object, object_list);
        }
    }
}


#endif // REFINED_CONES_H

