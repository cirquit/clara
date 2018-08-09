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
#include "clara-util.h"

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
            //! noisy `x`, `y` position and relative `x`, `y` position rotated by the respective yaw at the time (normalized)
            using raw_cone_data = std::tuple<T, T, T, T>;
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
            , _apply_variance_step_count(apply_variance_step_count)
            , _variance_xx(variance_xx)
            , _variance_yy(variance_yy)
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
                if (new_cones.empty()) {
                    no_observations = true;
                    return _cone_states;
                }
                // set flag to allow access to _detected_cluster_ixs_copy
                no_observations = false;
                // when we start the car, we will take every visible cone as a new cluster, we have the assumption that we don't see cones twice in a single image
                if (_cone_states.size() == 0)
                {   
                    for(size_t c_ix = 0; c_ix < new_cones.size(); ++c_ix)
                    {
                        const raw_cone_data & cone = new_cones[c_ix];
                        // some red cones are detected too close to each other, therefore we want to merge them. We iterate over all cones and check for distance
                        bool not_too_close = true;
                        for (size_t oc_ix = 0; oc_ix < new_cones.size(); ++oc_ix)
                        {
                            // if it's the same cone, we don't compare it
                            if (c_ix == oc_ix) continue;
                            if (c_ix  < oc_ix) continue;
                            const raw_cone_data & other_cone = new_cones[oc_ix];
                            double distance = util::euclidean_distance<double>(cone, other_cone);
                            // if it's is too close (< 0.5), we don't want to add it
                            if (distance < 1.5) { not_too_close = false; }
                        }
                        if (not_too_close)
                        {
                            _add_new_cluster_with_ob(cone);
                        }
                    }
                }
                // otherwise, we need to detect new cones and associate current clusters to our observations
                else
                {
                    for(auto cone : new_cones){
                        cluster_it prob_cluster_it = _get_most_probable_cluster_it(cone);
                        if ((*prob_cluster_it).distance_greater_than(cone, _max_dist_btw_cones_m))
                        {
                            _add_new_cluster_with_ob(cone);
                        }
                        // if it's not below the threshold, we need to put it in an existing cluster
                        else
                        {
                            _add_observation(prob_cluster_it, cone);
                        }
                    }
                }
                // finished with detection, clear for the next step
                // copy detected cluster from current step for search bounds calculation
                _detected_cluster_ix_copy.clear();
                std::copy(_detected_cluster_ix.begin(), _detected_cluster_ix.end(), std::back_inserter(_detected_cluster_ix_copy));
                _detected_cluster_ix.clear();
                return _cone_states;
            }

            /** \brief
              *
              */ 
            std::vector< std::tuple< double, double > > estimate_positional_difference(const std::vector<raw_cone_data> & new_cones)
            {

                std::vector< std::tuple< double, double > > position_differences;
                for(auto cone : new_cones){
                    cluster_it prob_cluster_it = _get_most_probable_cluster_it(cone);
                    if ((*prob_cluster_it).distance_greater_than(cone, _max_dist_btw_cones_m))
                    {
                        // do nothing..., we didn't 
//                        std::cout << "Was too far away!\n";
                    }
                    else
                    {   
//                        std::cout << "Matched a cluster!\n";
                        // if we got a "sure" match to a cluster
                        position_differences.push_back((*prob_cluster_it).difference(cone));
//                        std::cout << "   diff: " << std::get<0>(position_differences.back()) << ", "
//                                                 << std::get<1>(position_differences.back()) << '\n';
                        _add_detected_cone_ix(prob_cluster_it);
                    }
                }
                // finished with detection, clear for the next step
                // copy detected cluster from current step for search bounds calculation
                if (!_detected_cluster_ix.empty()){
                    _detected_cluster_ix_copy.clear();
                    std::copy(_detected_cluster_ix.begin(), _detected_cluster_ix.end(), std::back_inserter(_detected_cluster_ix_copy));
                    _detected_cluster_ix.clear();
                }
                return position_differences;

            }

            //! Getter for the current clusters
            constexpr std::vector<cone_state<T>> & get_cluster()
            {
                return _cone_states;
            }

            //! if there was no data observed this step, return an empty list of "used" clusters
            const std::vector<size_t> & get_detected_cluster_ixs() const
            {
                return no_observations ? _detected_cluster_ix_empty : _detected_cluster_ix_copy;
            }


            //! allocates a new vector and returns all currently "seen" cluster **BAD for performance**
            std::vector<cone_state<T>> get_detected_cluster() const
            {
                const std::vector<size_t> & detected_cluster_ix = get_detected_cluster_ixs();
                std::vector<cone_state<double>> cone_vec;
                cone_vec.reserve(detected_cluster_ix.size());
                clara::util::for_each_(detected_cluster_ix, [&](size_t ix){ cone_vec.emplace_back(_cone_states[ix]); });
                return cone_vec;
            }

            //! calls get_detected_cluster, sorts them by nearest to the car position (x,y) **BAD for performance**
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

            //! prints the cluster to the std::cout as a valid python library
            void print_data_assoc ( int color ) {
                const std::vector<clara::cone_state<T>> & cluster = get_cluster();
                const double cluster_weight = 0; // deprecated, numpy needs this value to have a valid matrix \todo fix python script

                if (color == 0) { std::cout << "yellow_cone_data = np.array([\n"; }
                if (color == 1) { std::cout << "blue_cone_data = np.array([\n"; }
                if (color == 2) { std::cout << "red_cone_data = np.array([\n"; }
                clara::util::for_each_(cluster, [&](const clara::cone_state<double> & cs)
                {
                    double mean_x = cs._mean_vec[0];
                    double mean_y = cs._mean_vec[1];
                    double cov_xx = cs._cov_mat[0];// - 0.45;
                    double cov_xy = cs._cov_mat[1];
                    double cov_yy = cs._cov_mat[3];// - 0.45;

                    std::string np_b = "np.array([";
                    std::string np_e = "])";
                    if ( mean_x != 0 || mean_y != 0 ) {
                        std::cout << np_b
                                  << "["  << mean_x << ", " << mean_y << "]"  << ", "
                                  << "[[" << cov_xx << ", " << cov_xy << "]" << ", "
                                  << "["  << cov_xy << ", " << cov_yy << "]]" << ", "
                                  << "["  << cluster_weight << "]"
                                  << np_e << ",\n"; 
                    }
                });
                std::cout << "]);\n";
            }

        //! prints the observations to the std::cout as a valid python library
        void print_observations( int color )
        {
            const std::vector<clara::cone_state<T>> & cluster = get_cluster();

            if (color == 0) { std::cout << "yellow_obs_cone_data = np.array([\n"; }
            if (color == 1) { std::cout << "blue_obs_cone_data = np.array([\n"; }
            if (color == 2) { std::cout << "red_obs_cone_data = np.array([\n"; }
            clara::util::for_each_(cluster, [&](const clara::cone_state<double> & cs)
            {
                double mean_x = cs._mean_vec[0];
                double mean_y = cs._mean_vec[1];

                if ( mean_x != 0 || mean_y != 0 ) {
                    for(auto obs : cs._observations)
                    {
                        std::cout << "[" << obs[0] << ", " << obs[1] << "],\n"; 
                    }
                }
            });
            std::cout << "]);\n";
        }

        //! If we want to restart the clustering, we reset every state this class has like _cone_states, _pdfs and _detected_cluster_ix
        void reset_state()
        {
            _cone_states.clear();
            _pdfs.clear();
            _detected_cluster_ix.clear();
            _detected_cluster_ix_copy.clear();
            no_observations = false;
        }


        // methods
        private:
            
            //! returns the iterator at the element with the highest association probability based on the pdf \todo single pass min-max
            cluster_it _get_most_probable_cluster_it(const raw_cone_data & cone)
            {   
                auto ixs = get_detected_cluster_ixs();
                int min_ix = static_cast<int>(*std::min_element(ixs.begin(), ixs.end()));
                int max_ix = static_cast<int>(*std::max_element(ixs.begin(), ixs.end()));
                
                // ugly looping of indices, adapted for future possibilty of #pramga omp fun - needs to be a ringbuffer
                // this is tested in tests/looping_tests.cc
                int distance = max_ix - min_ix + 2 * _cluster_search_range;
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

            //! updated the cone with an observation, saves the index for the next step
            void _add_observation(typename std::vector<cone_state<T>>::iterator & it, const raw_cone_data & observation)
            {
                (*it).add_observation(observation);
                (*it).update_state();
                _add_detected_cone_ix(it);
            }

            //! calculating the index of the detected cone and adds this into the _detected_cluster_ix container. Won't be added if it's already detected
            void _add_detected_cone_ix(const typename std::vector<cone_state<T>>::iterator & it)
            {
                auto ix = std::distance(_cone_states.begin(), it);
            //    auto maybe_ix_it = std::find(_detected_cluster_ix.begin(), _detected_cluster_ix.end(), ix);
//                // if it's not detected, add it
            //    if (maybe_ix_it == _detected_cluster_ix.end())
            //    {
                    _detected_cluster_ix.push_back(ix);
            //    }
            }

        // member (were private, but for logging purposes we mage them public)
        public: 
            //! saving all cluster here
            std::vector<cone_state<T>> _cone_states;
            //! \todo
            size_t _max_dist_btw_cones_m;
            //! how often should we apply additional variance in x,y dim
            size_t _apply_variance_step_count;
            //! what cluster is the last observed, we use that to span the association window 
            size_t _current_cone_ix;
            //! additional variance for sparse clusters in x
            double _variance_xx;
            //! additional variance for sparse clusters in y
            double _variance_yy;
            //! how far do we search infront and back of the min/max index of _detected_cluster_ix
            int _cluster_search_range;
            //! temporary storage for all pdf scores
            std::vector<double>  _pdfs;
            //! holds all currently used clusters, is filled in for classify_new_data
            std::vector<size_t>  _detected_cluster_ix;
            //! holds all currently used clusters, copied from the last step and overwritten at the end of classify_new_data. used to hold windowed bounds for cluster
            std::vector<size_t>  _detected_cluster_ix_copy;
            //! because we return everyting by const reference, we need to have a empty static member for get_detected_cluster_ixs
            std::vector<size_t>  _detected_cluster_ix_empty;
            //! because we need to hold our _detected_cluster_ix_copy for the next classification, we need to be able to distinguish "no detected cones" steps
            bool no_observations = false;
    };

} // namespace clara

/*! @} End of Doxygen Groups*/
#endif // DATA_ASSOCIATION_H
