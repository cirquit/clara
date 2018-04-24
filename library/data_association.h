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
    /** \brief A class which handles the data association task for CLARA
     *  
     *  Cones are observed by their absolute position (x,y), a GMM is used to cluster them into the best possible positional approximation
     * 
     *  Template arguments:
     *  * `T` = type of the cones (has to support floating point caluclationsfloat / double)
     */ 
    template <class T>
    class data_association
    {
        public:
            //! for convenience purposes
            using self_t        = data_association<T>;
            //! noisy `x`, `y` position
            using raw_cone_data = std::tuple<T, T>;
            //! iterator for _cone_states
            using cluster_it    = typename std::vector<cone_state<T>>::iterator;
        // constructors
        public:
            /** \brief Default constructor which allocates multiple preallocated_cluster_count-sized vectors
              * Currently we use add a some variance in both x and y directions until several observations have been made to allow better association
              * Args:
              *   * preallocated_cluster_count - how many cluster do we want to allocate
              *   * detected_cones_per_step    - how many cluster do we match on runtime (preallocation of index-vector)
              *   * max_dist_btwn_cones        - maximum distance between cones where we cut off
              *   * variance_xx                - applied variance in X dimension for apply_variance_step_count to the clusters
              *   * variance_yy                - applied variance in Y dimension for apply_variance_step_count to the clusters
              *   * apply_variance_step_count  - applies variance for this amount of steps to every new 
              *   * cluster_search_range       - how far to do we look for infront or behind us for similar clusters (min_ix - c_s_r | max_ix + c_s_r)
              */
            data_association(size_t preallocated_cluster_count
                           , size_t preallocated_detected_cones_per_step
                           , double max_dist_btw_cones_m
                           , double variance_xx
                           , double variance_yy
                           , size_t apply_variance_step_count
                           , int    cluster_search_range)
            : _max_dist_btw_cones_m(max_dist_btw_cones_m)
            , _variance_xx(variance_xx)
            , _variance_yy(variance_yy)
            , _apply_variance_step_count(apply_variance_step_count)
            , _cluster_search_range(cluster_search_range)
            {
                _cone_states.reserve(preallocated_cluster_count);
                _detected_cluster_ix.reserve(preallocated_detected_cones_per_step);
                _detected_cluster_ix_copy.reserve(preallocated_detected_cones_per_step);
            };

            //! deleted copy constructor, we don't want anybody to move or copy this object
            data_association(const self_t & other)  = delete;
            //! deleted move constructor, we don't want anybody to move or copy this object
            data_association(const self_t && other) = delete;

        // methods
        public:
            /** \brief Creates new clusters based on low PDF score on all clusters, otherwise appends to the one with the highest probability
              * Returns:
              *    * current state of all clusters
              */
            const std::vector<cone_state<T>> &
            classify_new_data(const std::vector<raw_cone_data> & new_cones)
            {  

                // if somehow we get no observations, we don't do anything
                if (new_cones.empty()) return _cone_states;

                // copy detected cluster from previous steps for search bounds calculation
                std::copy(_detected_cluster_ix.begin(), _detected_cluster_ix.end(), std::back_inserter(_detected_cluster_ix_copy));
                _detected_cluster_ix.clear();

                // when we start the car, we will take every visible cone as a new cluster, we have the assumption that we don't see cones twice in a single image
                if (_cone_states.size() == 0)
                {
                    for(auto cone : new_cones)
                    {
                        _add_new_cluster_with_ob(cone);
                    }
                }
                // otherwise, we need to detect new cones and associate current clusters to our observations
                else
                {
                    for(auto cone : new_cones){
                        auto cluster_it = _get_most_probable_cluster_it(cone);
                        if ((*cluster_it).distance_greater_than(cone, _max_dist_btw_cones_m))
                        {
                            _add_new_cluster_with_ob(cone);
                        }
                        // if it's not below the threshold, we need to put it in an existing cluster
                        else
                        {   
                            _add_observation(cluster_it, cone);
                        }
                    }
                }
                // finished with detection, clear for the next step
                _detected_cluster_ix_copy.clear();
                return _cone_states;
            }

            //! Getter for the current clusters
            constexpr std::vector<cone_state<T>> & get_cluster()
            {
                return _cone_states;
            }

            //! convenient naming
            const std::vector<size_t> & get_detected_cluster_ixs() const
            {
                return _detected_cluster_ix;
            }

            //! convenient naming
            const std::vector<size_t> & get_previously_detected_cluster_ixs() const
            {
                return _detected_cluster_ix_copy;
            }


            //! allocates a new vector and returns all currently "seen" cluster
            std::vector<cone_state<T>> get_detected_cluster() const
            {
                const std::vector<size_t> & detected_cluster_ix = get_detected_cluster_ixs();
                std::vector<cone_state<double>> cone_vec;
                cone_vec.reserve(detected_cluster_ix.size());
                clara::util::for_each_(detected_cluster_ix, [&](size_t ix){ cone_vec.emplace_back(_cone_states[ix]); });
                return cone_vec;
            }

            //! calls get_detectd_cluster, sorts them by nearest to the car position (x,y)
            const std::vector<cone_state<T>> get_detected_cluster_sorted(const std::tuple<double, double> car_pos) const
            {
                std::vector<cone_state<T>> detected_cluster = get_detected_cluster();
                std::sort(detected_cluster.begin(), detected_cluster.end(), [&](const cone_state<T> & a, const cone_state<T> & b)
                    {
                        const double dist_A = a.distance(car_pos);
                        const double dist_B = b.distance(car_pos);
                        return dist_A < dist_B;
                    });
                return detected_cluster;
            }

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
                return _cone_states.size();
            }

        // methods
        private:
            
            //! returns the iterator at the element with the highest association probability based on the pdf \todo single pass min-max
            cluster_it _get_most_probable_cluster_it(const raw_cone_data & cone)
            {   

                auto ixs = get_previously_detected_cluster_ixs();
                int min_ix = static_cast<int>(*std::min_element(ixs.begin(), ixs.end()));
                int max_ix = static_cast<int>(*std::max_element(ixs.begin(), ixs.end()));
                
                // ugly looping of indices, adapted for future possibilty of #pramga omp fun - needs to be a ringbuffer
                int distance = max_ix - min_ix + 2 * _cluster_search_range;
                int max_elem = 0;
                int ix       = 0;
                int min_diff = min_ix - _cluster_search_range;
                if (min_diff < 0)
                    ix = std::max(0, static_cast<int>(_cone_states.size()) + min_diff);
                else
                    ix = min_diff;

                int best_cone_ix = ix;
                for(int i = 0; i < distance; i++)
                {
                    int my_ix = (ix + i) % _cone_states.size();
                    if(_cone_states[best_cone_ix].pdf(cone) < _cone_states[my_ix].pdf(cone))
                        best_cone_ix = my_ix;
                }

                return _cone_states.begin() + best_cone_ix;

                // // partial application of _compare_clusters_by_probability
                // std::function<bool(cone_state<T>&, cone_state<T>&)> cmp = [&](cone_state<T> & a
                //                                                           , cone_state<T> & b)
                // {
                //     return _compare_clusters_by_probability(cone, a, b);
                // };

                // // default init to compare them later
                // cluster_it min_cluster = _cone_states.begin();
                // cluster_it max_cluster = _cone_states.begin();

                // // Case 1: min overflows into negative (bounds too big, or min is too small)
                // int min_diff = min_ix - _cluster_search_range;
                // if (min_diff < 0)
                // {
                //     cluster_it min_bnds_01 = std::max_element(_cone_states.begin(),          _cone_states.begin() + min_ix + 1,
                //          [&](cone_state<T> & a, cone_state<T> & b)
                //          {
                //              return a.pdf(cone) < b.pdf(cone); 
                //          }
                //     );
                //     cluster_it min_bnds_02 = std::max_element(_cone_states.end() - min_diff, _cone_states.end(),
                //          [&](cone_state<T> & a, cone_state<T> & b)
                //          {
                //              return a.pdf(cone) < b.pdf(cone); 
                //          }
                //     );
                //     min_cluster = _return_cluster_by_probability(cone, min_bnds_01, min_bnds_02);
                // } 
                // // Case 2: min doesn't overflow
                // else
                // {
                //     min_cluster = std::max_element(_cone_states.begin() + min_diff, _cone_states.begin() + min_ix + 1,
                //          [&](cone_state<T> & a, cone_state<T> & b)
                //          {
                //              return a.pdf(cone) < b.pdf(cone); 
                //          }
                //     );
                // }

                // // Case 3: max overflows into size-1 (bounds too big, or max too big)
                // int max_sum  = max_ix + _cluster_search_range;
                // int max_diff = max_sum - (_cone_states.size()-1);
                // if (max_diff > 0)
                // {
                //     cluster_it max_bnds_01 = std::max_element(_cone_states.begin() + max_ix, _cone_states.end(),
                //          [&](cone_state<T> & a, cone_state<T> & b)
                //          {
                //              return a.pdf(cone) < b.pdf(cone); 
                //          }
                //     );
                //     cluster_it max_bnds_02 = std::max_element(_cone_states.begin(),          _cone_states.begin() + max_diff + 1,
                //          [&](cone_state<T> & a, cone_state<T> & b)
                //          {
                //              return a.pdf(cone) < b.pdf(cone); 
                //          }
                //     );
                //     max_cluster = _return_cluster_by_probability(cone, max_bnds_01, max_bnds_02);
                // }
                // // Case 4: max doesn't overflow
                // else
                // {
                //     max_cluster = std::max_element(_cone_states.begin() + max_ix, _cone_states.begin() + max_sum + 1,
                //          [&](cone_state<T> & a, cone_state<T> & b)
                //          {
                //              return a.pdf(cone) < b.pdf(cone); 
                //          }
                //     );
                // }

                // return _return_cluster_by_probability(cone, min_cluster, max_cluster);
                // return std::max_element(_cone_states.begin(), _cone_states.end(), [&](cone_state<T> & a, cone_state<T> & b)
                // {
                //     return a.pdf(cone) < b.pdf(cone);
                // });
            }

            //! \todo
            bool _compare_clusters_by_probability(const raw_cone_data & cone, cone_state<T> & a, cone_state<T> & b)
            {
                return a.pdf(cone) < b.pdf(cone);
            }

            //! \todo
            cluster_it _return_cluster_by_probability(const raw_cone_data & cone, cluster_it a, cluster_it b)
            {
                return _compare_clusters_by_probability(cone, *a, *b) ? a : b;
            }

            /** creates a default cluster with _variance_xx, _variance_yy and _apply_variance_step_count at the end of _cone_states
             *  adds the observation and updates it
             */
            void _add_new_cluster_with_ob(const raw_cone_data & observation)
            {
                _cone_states.emplace_back(cone_state<T>(_variance_xx, _variance_yy, _apply_variance_step_count));
                typename std::vector<cone_state<T>>::iterator last_it = std::prev(_cone_states.end()); 
                _add_observation(last_it, observation);
            }

            //! updated the cone with an observation, adds the 
            void _add_observation(typename std::vector<cone_state<T>>::iterator & it, const raw_cone_data & observation)
            {
                (*it).add_observation(observation);
                (*it).update_state();
                _add_detected_cone_ix(it);
            }

            //! convenient naming
            void _add_detected_cone_ix(const typename std::vector<cone_state<T>>::iterator & it)
            {
                auto ix = std::distance(_cone_states.begin(), it);
                _detected_cluster_ix.push_back(ix);
            }

        // member (were private, but for logging purposes we mage them public)
        public: 
            //! saving all cluster here
            std::vector<cone_state<T>> _cone_states;
            //! additional variance for sparse clusters in x
            double _variance_xx;
            //! additional variance for sparse clusters in y
            double _variance_yy;
            //! how often should we apply additional variance in x,y dim
            size_t _apply_variance_step_count;
            //! what cluster is the last observed, we use that to span the association window 
            size_t _current_cone_ix;
            //! \todo
            size_t _max_dist_btw_cones_m;
            //! temporary storage for all pdf scores
            std::vector<double>  _pdfs;
            //! holds all currently used clusters, needed for future analysis
            std::vector<size_t>  _detected_cluster_ix;
            //! holds all previously used clusters, needed for bounds caluclation from previous steps
            std::vector<size_t>  _detected_cluster_ix_copy;
            //! how far do we search infront and back of the min/max index of _detected_cluster_ix
            int _cluster_search_range;

    };

} // namespace clara

/*! @} End of Doxygen Groups*/
#endif // DATA_ASSOCIATION_H
