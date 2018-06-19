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

    /** \brief Find the two nearest cones in the seen ixs. `.first` is the nearest, `.second` the second nearest
      *
      */ 
    std::pair<size_t, size_t> get_nearest_cones(const std::vector<size_t>  & ixss
                                              , std::vector<cone_state<double>> & cluster
                                              , const std::tuple<double, double> & pos)
    {
        UNUSED(ixss);
        std::vector<size_t> ixs;
        ixs.reserve(cluster.size());
        size_t counter = 0;
        std::for_each(cluster.begin(), cluster.end(), [&](cone_state<double> cl){ UNUSED(cl); ixs.push_back(counter++); });

        // find the nearest blue cone, save index and distance
        size_t ix_01       = ixs[0];
        double distance_01 = cluster[ix_01].distance(pos);
        size_t ix_02       = ixs[1];
        double distance_02 = cluster[ix_02].distance(pos);

        std::pair<double, size_t> nearest_cluster;
        std::pair<double, size_t> sec_nearest_cluster;
        // sort the two cluster by distance
        if (distance_01 < distance_02) {
            nearest_cluster     = std::make_pair(distance_01, ix_01);
            sec_nearest_cluster = std::make_pair(distance_02, ix_02);
        } else {
            nearest_cluster     = std::make_pair(distance_02, ix_02);
            sec_nearest_cluster = std::make_pair(distance_01, ix_01);
        }
        // iterate in O(n) through all but the first element and get the two nearest cluster
        std::for_each(ixs.begin() + 1, ixs.end(), [&](const size_t & ix)
        {
            distance_01 = cluster[ix].distance(pos);
            if (nearest_cluster.first > distance_01)
            {
                sec_nearest_cluster.first  = nearest_cluster.first;
                sec_nearest_cluster.second = nearest_cluster.second;
                nearest_cluster.first = distance_01;
                nearest_cluster.second = ix;
            } else if (sec_nearest_cluster.first > distance_01)
            {
                sec_nearest_cluster.first  = distance_01;
                sec_nearest_cluster.second = ix;
            }
        });

        return std::make_pair(nearest_cluster.second, sec_nearest_cluster.second);
    }


    maybe_cones get_basecase_cones(const std::tuple< double, double > & pos
                                 , clara::data_association< double > & yellow_data_association
                                 , clara::data_association< double > & blue_data_association)
    {
        // get cluster by reference to not change ownership
        auto & yellow_cluster = yellow_data_association.get_cluster();
        auto & blue_cluster   = blue_data_association.get_cluster();
        //
        std::vector<size_t> yellow_detected_cluster_ix;
        std::vector<size_t> blue_detected_cluster_ix;
        
        near_cones y_cs;
        near_cones b_cs;

        if (yellow_cluster.size() >= 2)
        {
            // find the two nearest yellow cones
            auto near_yellow_ixs = get_nearest_cones(yellow_detected_cluster_ix, yellow_cluster, pos);
            // 
            cone_position y_c_01;
            y_c_01[0] = yellow_cluster[near_yellow_ixs.first]._mean_vec[0];
            y_c_01[1] = yellow_cluster[near_yellow_ixs.first]._mean_vec[1];
            cone_position y_c_02;
            y_c_02[0] = yellow_cluster[near_yellow_ixs.second]._mean_vec[0];
            y_c_02[1] = yellow_cluster[near_yellow_ixs.second]._mean_vec[1];
            y_cs = {{ y_c_01, y_c_02 }};
        }
        if (blue_cluster.size() >= 2)
        {
            // find the two nearest blue cones
            auto near_blue_ixs   = get_nearest_cones(blue_detected_cluster_ix, blue_cluster, pos);
            //
            cone_position b_c_01;
            b_c_01[0] = blue_cluster[near_blue_ixs.first]._mean_vec[0];
            b_c_01[1] = blue_cluster[near_blue_ixs.first]._mean_vec[1];
            cone_position b_c_02;
            b_c_02[0] = blue_cluster[near_blue_ixs.second]._mean_vec[0];
            b_c_02[1] = blue_cluster[near_blue_ixs.second]._mean_vec[1];
            b_cs = {{ b_c_01, b_c_02 }};
            return concept::maybe<cones_tuple>(std::make_tuple(y_cs, b_cs));
        } else {
            return concept::maybe<cones_tuple>();
        }
    }

    // /** \brief \todo
    //   * 
    //   */
    // maybe_cones get_basecase_cones(const std::tuple<double, double>  & pos
    //                              , clara::data_association< double > & yellow_data_association
    //                              , clara::data_association< double > & blue_data_association)
    // {
    //     // get cluster by reference to not change ownership
    //     auto & yellow_cluster = yellow_data_association.get_cluster();
    //     auto & blue_cluster   = blue_data_association.get_cluster();
    //     // get currently seen cluster indexes by reference
    //     const auto & yellow_detected_cluster_ix = yellow_data_association.get_detected_cluster_ixs();
    //     const auto & blue_detected_cluster_ix   = blue_data_association.get_detected_cluster_ixs();

    //     std::vector<size_t> yellow_detected_cluster_ix_copy;
    //     std::vector<size_t> blue_detected_cluster_ix_copy;
    //     std::copy(yellow_detected_cluster_ix.begin(), yellow_detected_cluster_ix.end(),
    //           std::back_inserter(yellow_detected_cluster_ix_copy));
    //     std::copy(blue_detected_cluster_ix.begin(), blue_detected_cluster_ix.end(),
    //           std::back_inserter(blue_detected_cluster_ix_copy));

    //     if (yellow_detected_cluster_ix_copy.size() > 0)
    //     {
    //         size_t min_y_ix = *std::min_element(yellow_detected_cluster_ix_copy.begin()
    //                                           , yellow_detected_cluster_ix_copy.end());
    //         for(int i = 1; i < 3; ++i)
    //         {
    //             int y_ix = static_cast<int>(min_y_ix) - i;
    //             if (y_ix > -1) // \todo looping
    //             {
    //                 yellow_detected_cluster_ix_copy.push_back(static_cast<size_t>(y_ix));
    //             }
    //         }
    //     }

    //     if (blue_detected_cluster_ix_copy.size() > 0)
    //     {
    //         size_t min_b_ix = *std::min_element(blue_detected_cluster_ix_copy.begin()
    //                                           , blue_detected_cluster_ix_copy.end());
    //         for(int i = 1; i < 3; ++i)
    //         {
    //             int b_ix = static_cast<int>(min_b_ix) - i;
    //             if (b_ix > -1) // \todo looping
    //             {
    //                 blue_detected_cluster_ix_copy.push_back(static_cast<size_t>(b_ix));
    //             }
    //         }
    //     }

    //     // if we have at least detected one cones for each color
    //    if (yellow_detected_cluster_ix_copy.size() >= 2
    //        && blue_detected_cluster_ix_copy.size() >= 2)
    //     {
    //         // find the two nearest yellow cones
    //         auto near_yellow_ixs = get_nearest_cones(yellow_detected_cluster_ix_copy, yellow_cluster, pos);
    //         // find the two nearest blue cones
    //         auto near_blue_ixs   = get_nearest_cones(blue_detected_cluster_ix_copy, blue_cluster, pos);

    //         // fill the types
    //         cone_position y_c_01;
    //         y_c_01[0] = yellow_cluster[near_yellow_ixs.first]._mean_vec[0];
    //         y_c_01[1] = yellow_cluster[near_yellow_ixs.first]._mean_vec[1];
    //         cone_position y_c_02;
    //         y_c_02[0] = yellow_cluster[near_yellow_ixs.second]._mean_vec[0];
    //         y_c_02[1] = yellow_cluster[near_yellow_ixs.second]._mean_vec[1];
    //         near_cones y_cs = {{ y_c_01, y_c_02 }};

    //         cone_position b_c_01;
    //         b_c_01[0] = blue_cluster[near_blue_ixs.first]._mean_vec[0];
    //         b_c_01[1] = blue_cluster[near_blue_ixs.first]._mean_vec[1];
    //         cone_position b_c_02;
    //         b_c_02[0] = blue_cluster[near_blue_ixs.second]._mean_vec[0];
    //         b_c_02[1] = blue_cluster[near_blue_ixs.second]._mean_vec[1];
    //         near_cones b_cs = {{ b_c_01, b_c_02 }};

    //         if (euclidean_distance(std::make_tuple(b_c_01[0], b_c_01[1]),
    //                                std::make_tuple(y_c_01[0], y_c_01[1])) > 4) return concept::maybe<cones_tuple>();

    //         if (euclidean_distance(std::make_tuple(b_c_02[0], b_c_02[1]),
    //                    std::make_tuple(y_c_02[0], y_c_02[1])) > 4) return concept::maybe<cones_tuple>();

    //         return concept::maybe<cones_tuple>(std::make_tuple(y_cs, b_cs));
    //     } else {
    //         return concept::maybe<cones_tuple>();
    //     }
    //     return concept::maybe<cones_tuple>();
    // }

} // namespace util
} // namespace clara

#endif // SEARCH_CONES_H
