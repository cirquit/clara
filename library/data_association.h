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

#ifndef DATA_ASSOCIATION_H
#define DATA_ASSOCIATION_H

#include <functional>
#include <iostream>
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

    /** \brief A class which handles the data association task for CLARA
     *
     *  The task of clara is to return the *best* estimate of the current world position
     *  
     *  To do this, we detect cones based on some sensors which return:
     *  * **relative distance** to the current position
     *  * **yaw angle** to the car direction. This is in rad, positive means it's left of us, negative means right \todo create a nice image
     * 
     *  Cones are saved by their absolute position (x,y), based on the starting point of the car (usually `(0,0)`)
     * 
     *  Template arguments:
     *  * `N` = maximum amount of cones which we'll be able to identify
     */ 
    template<size_t N> 
    class data_association
    {
        public:
            //! for convenience purposes
            using self_t        = data_association<N>;
            //! noisy `x`, `y` position
            using raw_cone_data = std::tuple<double, double>;

        // constructors
        public:
            /** \brief Default constructor which allocates multiple N-sized arrays
              */
            data_association()
            : _detected_cones {0}
            , _pdfs {}
            , _cone_state_weights {}
            , _probs {}
            , _cone_states {} { };

            //! deleted copy constructor, we don't want anybody to move or copy this object
            data_association(const self_t & other)  = delete;

            //! deleted move constructor, we don't want anybody to move or copy this object
            data_association(const self_t && other) = delete;

        // methods
        public:
            /** \brief Creates new clusters based on low PDF score on all clusters, otherwise appends to the one with the highest probability
              * Returns:
              *    * current state of all clusters, \todo maybe return all used ones with _detected_cones
              */
            const std::array<cone_state<double>, N> &
            classify_new_data(const std::vector<raw_cone_data> & new_cones)
            {
                // when we start the car, we will take every visible cone as a ground truth cluster, we have the assumption that we don't see cones twice in a single image
                if (_detected_cones == 0)
                {
                    for(auto cone : new_cones)
                    {
                        _cone_states[_detected_cones].add_observation(cone);
                        _cone_states[_detected_cones].update_state();
                        _detected_cones++;
                    }
                }
                // otherwise, we need to detect new cones and associate current clusters to our observations
                else
                {
                    // \todo, only check the last m cones for cluster association
                    // size_t m = 5; // need to iterate not from 0 but from max(0, min(m, detected_copy_cones - 5))

                    // the probability is weighted based on the sum of all other cones probabilities
                    double            probs_sum = 0;
                    // compute the cluster weights (how many observations each one has divided by all)
                    auto           cone_weights = get_cone_state_weights();
                    // only check for previously created clusters, not the ones we create now
                    size_t _detected_cones_copy = _detected_cones;
                    // iterate over all the cones
                    for(auto cone : new_cones){

                        std::cout << "Detected cone: (" << std::get<0>(cone) << ", "
                                                        << std::get<1>(cone) << ")\n";
                        // for all clusters, calculate their pdfs and probabilities
                        for (size_t i = 0; i < _detected_cones_copy; ++i)
                        {
                            _pdfs[i]  = _cone_states[i].pdf(cone);
                            _probs[i] = _cone_states[i].pdf(cone) * cone_weights[i];
                            probs_sum += _probs[i];
                        }

                        // log every calculation, this is dependent on probs_sum
                        for (size_t i = 0; i < _detected_cones_copy; ++i)
                        {
                            std::cout << "    pdf  ["   << i << "]: " << _pdfs[i]
                                      << "    probs["   << i << "]: " << _probs[i] / probs_sum * 100 << '%'
                                      << "    weights[" << i << "]: " << _cone_state_weights[i] << '\n';
                        }

                        // find the maximum pdf value, if it's below our threshold, it's a new cluster \todo test this rigorously 
                        auto it = std::max_element(_pdfs.begin(), _pdfs.end());
                        if (*it < 0.0001)
                        {
                            std::cout << "    New cluster detected, adding to list\n";
                            _cone_states[_detected_cones].add_observation(cone);
                            _cone_states[_detected_cones].update_state();
                            _detected_cones++;
                        }
                        // if it's not below the threshold, we need to put it in an existing cluster
                        else
                        {    
                            // get highest probability of all clusters - normally we would need to filter those out one by one for the next cones \todo
                            auto prob_it = std::max_element(_probs.begin(), _probs.end());
                            // get the index
                            size_t prob_ix = std::distance(_probs.begin(), prob_it);
                            // add the cone to the clusters
                            _cone_states[prob_ix].add_observation(cone);
                            _cone_states[prob_ix].update_state();
                            std::cout << "    Added cone to this cluster Nr." << prob_ix
                                      << " with mean: (" << _cone_states[prob_ix]._mean_vec[0] << ", "
                                                         << _cone_states[prob_ix]._mean_vec[1] << ")\n";
                        }
                    }
                }
                return _cone_states;
            }

        // methods
        private:

            /** \brief Returns the count of all seen cones which is described in the the competition handbook DE7.4 (2018)
             *  This assumes that every visible object is a cone and **we don't have outliers**
             *  \todo Current runtime complexity *O(N)*
             */ 
            constexpr size_t get_cone_count_all()
            {
                return std::accumulate(_cone_states.begin(), _cone_states.end(), 0,
                    [](size_t acc, cone_state<double> & cs)
                    {
                        return acc + cs.get_observations_size();
                    });
            }

            //! Returns the count of actual detected cones without duplicates, which is described in the competition handbook DE7.4 (2018)
            constexpr size_t get_cone_count_actual()
            {
                return _detected_cones;
            }

            /** \brief Calculates the relative weight of each cluster based on their _observations.size()
              * Modifies:
              *     * _cone_state_weights
              * Returns:
              *     * _cone_state_weights
              *
              * \todo Runtime complexity *O(2N)*
              */
            const std::array<double, N> & get_cone_state_weights()
            {   
                size_t cone_count_all = get_cone_count_all();
                util::enumerate(_cone_states.begin(), _cone_states.end(), 0, [&](unsigned i, cone_state<double> & cs)
                {
                    _cone_state_weights[i] = cs.get_observations_size() / cone_count_all;
                });
                return _cone_state_weights;
            }

        // member (were private, but for logging purposes we mage them public)
        public: 
            //! saving all cluster here (this might not be filled to the brink), _detected_cones saves how many are actually used
            std::array<cone_state<double>, N> _cone_states;
            //! which clusters are already filled
            size_t                 _detected_cones;
            //! temporary storage for all cluster weights (how many observations each one hase)
            std::array<double, N>  _cone_state_weights;
            //! temporary storage for all pdf scores
            std::array<double, N>  _pdfs;
            //! temporary storage for all probabilities (calculated by their pdf * cluster_weight)
            std::array<double, N>  _probs;
    };

} // namespace clara

/*! @} End of Doxygen Groups*/
#endif // DATA_ASSOCIATION_H