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

#ifndef SEARCH_CONES_H
#define SEARCH_CONES_H

#include "maybe.h"
#include "util.h"
/*!
 *  \addtogroup clara
 *  @{
 */

/** \brief CLARA namespace
 *
 */

namespace clara {

    namespace util { 

    //! a cone position (x,y)
    using cone_position = std::array<double, 2>;
    //! 2 cones with positional information
    using near_cones    = std::array<cone_position, 2>;
    //! two cones for each color
    using cones_tuple   = std::tuple<near_cones, near_cones>;
    //! maybe returns 4 cones, if we have enough objects to create it
    using maybe_cones   = typename concept::maybe<cones_tuple>;


    /** \brief Find the nearest cone in the seen ixs, returns the index of the nearest cone and deletes it from the index
      *
      */ 
    concept::maybe<size_t> get_nearest_cone(std::vector<size_t>  & ixs
                                          , std::vector<cone_state<double>> & cluster
                                          , const std::tuple<double, double> & pos)
    {
        if (ixs.empty()) return concept::maybe<size_t>();
        // find the nearest cone, save index and distance
        size_t ix_01       = ixs[0];
        double distance_01 = cluster[ix_01].distance(pos);

        std::tuple<double, size_t, size_t> nearest_cluster = std::make_tuple(distance_01, ix_01, 0);

        enumerate(ixs.begin(), ixs.end(), 0, [&](size_t vix, size_t ix){
        
            distance_01 = cluster[ix].distance(pos);
            if (std::get<0>(nearest_cluster) > distance_01)
            {
                std::get<0>(nearest_cluster) = distance_01;
                std::get<1>(nearest_cluster) = ix;
                std::get<2>(nearest_cluster) = vix;
            }
        });
        
        ixs.erase(ixs.begin() + std::get<2>(nearest_cluster));
        return concept::maybe<size_t>(std::get<2>(nearest_cluster));
    }


    /** \brief \todo
      * 
      */
    maybe_cones get_basecase_cones(const std::tuple<double, double>  & pos
                                 , clara::data_association< double > & yellow_data_association
                                 , clara::data_association< double > & blue_data_association)
    {
        // get cluster by reference to not change ownership
        auto & yellow_cluster = yellow_data_association.get_cluster();
        auto & blue_cluster   = blue_data_association.get_cluster();
        // get currently seen cluster indexes by reference
        const auto & yellow_detected_cluster_ix = yellow_data_association.get_detected_cluster_ixs();
        const auto & blue_detected_cluster_ix   = blue_data_association.get_detected_cluster_ixs();

        std::vector<size_t> yellow_detected_cluster_ix_copy;
        std::vector<size_t> blue_detected_cluster_ix_copy;
        std::copy(yellow_detected_cluster_ix.begin(), yellow_detected_cluster_ix.end(),
              std::back_inserter(yellow_detected_cluster_ix_copy));
        std::copy(blue_detected_cluster_ix.begin(), blue_detected_cluster_ix.end(),
              std::back_inserter(blue_detected_cluster_ix_copy));
        
        concept::maybe<size_t> m_front_yellow_cone = get_nearest_cone(yellow_detected_cluster_ix_copy
                                                      , yellow_cluster
                                                      , pos);
        concept::maybe<size_t> m_front_blue_cone = get_nearest_cone(blue_detected_cluster_ix_copy
                                                      , blue_cluster
                                                      , pos);

        if (yellow_detected_cluster_ix_copy.size() > 0)
        {
            size_t min_y_ix = *std::min_element(yellow_detected_cluster_ix_copy.begin()
                                              , yellow_detected_cluster_ix_copy.end());
            for(int i = 1; i < 3; ++i)
            {
                int y_ix = static_cast<int>(min_y_ix) - i;
                if (y_ix > -1) // \todo looping
                {
                    yellow_detected_cluster_ix_copy.push_back(static_cast<size_t>(y_ix));
                }
            }
        }

        if (blue_detected_cluster_ix_copy.size() > 0)
        {
            size_t min_b_ix = *std::min_element(blue_detected_cluster_ix_copy.begin()
                                              , blue_detected_cluster_ix_copy.end());
            for(int i = 1; i < 3; ++i)
            {
                int b_ix = static_cast<int>(min_b_ix) - i;
                if (b_ix > -1) // \todo looping
                {
                    blue_detected_cluster_ix_copy.push_back(static_cast<size_t>(b_ix));
                }
            }
        }

        concept::maybe<size_t> m_back_yellow_cone = get_nearest_cone(yellow_detected_cluster_ix_copy
                                                       , yellow_cluster
                                                       , pos);
        concept::maybe<size_t> m_back_blue_cone = get_nearest_cone(blue_detected_cluster_ix_copy
                                                       , blue_cluster
                                                       , pos);

        if (m_front_yellow_cone.has_value() &&
            m_front_blue_cone.has_value() &&
            m_back_yellow_cone.has_value() &&
            m_back_blue_cone.has_value())
        {
            if (m_front_yellow_cone.get_value() != m_back_yellow_cone.get_value() &&
                m_front_blue_cone.get_value() != m_back_blue_cone.get_value())
            {
                // fill the types
                cone_position y_c_01;
                size_t y_ix = m_front_yellow_cone.get_value();
                y_c_01[0] = yellow_cluster[y_ix]._mean_vec[0];
                y_c_01[1] = yellow_cluster[y_ix]._mean_vec[1];
                cone_position y_c_02;
                y_ix = m_back_yellow_cone.get_value();
                y_c_02[0] = yellow_cluster[y_ix]._mean_vec[0];
                y_c_02[1] = yellow_cluster[y_ix]._mean_vec[1];
                near_cones y_cs = {{ y_c_01, y_c_02 }};

                cone_position b_c_01;
                size_t b_ix = m_front_blue_cone.get_value();
                b_c_01[0] = blue_cluster[b_ix]._mean_vec[0];
                b_c_01[1] = blue_cluster[b_ix]._mean_vec[1];
                cone_position b_c_02;
                b_ix = m_back_blue_cone.get_value();
                b_c_02[0] = blue_cluster[b_ix]._mean_vec[0];
                b_c_02[1] = blue_cluster[b_ix]._mean_vec[1];
                near_cones b_cs = {{ b_c_01, b_c_02 }};

                return concept::maybe<cones_tuple>(std::make_tuple(y_cs, b_cs));
            } else {
                return concept::maybe<cones_tuple>();
            }
        }
        return concept::maybe<cones_tuple>();
    }

} // namespace util
} // namespace clara

#endif // SEARCH_CONES_H
