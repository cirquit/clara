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

#ifndef CONE_STATE_H
#define CONE_STATE_H

#include <functional>
#include <numeric>
#include <iostream>
#include <blaze/Math.h>
#include <cmath>
#include <math.h>
/*!
 *  \addtogroup clara
 *  @{
 */

/** \brief CLARA namespace
 *
 */
namespace clara {
    /** \brief A wrapper for the cone state
     *
     *  This class servers as the full knowledge base of a cone from the start of the detection
     *  We use this for the [data_association](data_association.h) task, which in turn uses the (Expectation Maximization algorithm)[https://en.wikipedia.org/wiki/Expectation%E2%80%93maximization_algorithm]
     *  to associate detected cones to the known cones. 
     *
     *  To do EM, we need to calculate the mean und covarianceiance matrix over `x, y`. Because we know we only have a 2-dimensional state, we provide
     *  the needed functionality to do a gauss-sampling. See `_get_inv_cov_mat()` and `_get_det_cov_mat()`
     *
     *  Because of the assumption that new cones 
     *
     *  Template arguments:
     *  * `T` is the type of the cone x,y coordinates 
     *
     *  Restrictions:
     *  * `T` has to be moveable and allow floating point calculations
     *
     * \todo add `is_integral` test for template `T`
     */
    template <typename T>
    class cone_state
    {
        public:
            //! for convenience purposes
            using self_t   = cone_state<T>;
            //! noisy `x`, `y` position and relative `x`, `y` position rotated by the respective yaw at the time (normalized) 
            using coords_t = std::array<T,4>;
        // constructors
        public:
            //! default constructor without noisy initialization
            cone_state() : cone_state(0, 0, 0) { }

            //! noisy constructor, we apply the _variance_xx and _variance_yy to the covarianceiance matrix until we have at least  seen _apply_variance_step_count observations
            cone_state(T variance_xx, T variance_yy, size_t apply_variance_step_count)
            : _modified(false)
            , _cov_mat {}
            , _inv_cov_mat {}
            , _det_cov_mat {}
            , _mean_vec {}
            , _variance_xx(variance_xx)
            , _variance_yy(variance_yy)
            , _apply_variance_step_count(apply_variance_step_count)
            {   // magic number (10 rounds à 10 times seeing the same cone)
                _observations.reserve(10 * 10);
            }

            //! allow copies
            cone_state(const self_t & other)  = default;

            //! allow moves
            // cone_state(self_t && other) = default;

        // methods
        public:

            //! Helper function for currying, calls add_observation(T x, T y, T x_rel, T y_rel)
            void add_observation(std::tuple<T, T, T, T> tup)
            {   
                add_observation(std::get<0>(tup)
                              , std::get<1>(tup)
                              , std::get<2>(tup)
                              , std::get<3>(tup));
            }

            /** \brief Adds an observation of coordiantes to the cone state
              *
              * This should be called if the EM algorithm determined with a sufficient probability that those coordiantes are in fact associated with this cone
              * For the thoughful people, we use `_set_modified()` and don't recalculate the `_cov_mat` and `_mean_vec` because we can have multiple observations of the same cone at the same time
              * This highly depends on the detection algorithm
              * 
              * \todo test streaming mean / covariance functions so we don't need to accumulate all observations
              */
            void add_observation(T x, T y, T x_rel, T y_rel)
            {   
                _observations.emplace_back( coords_t { { x, y, x_rel, y_rel } } );
                _set_modified(true);
            }

            //! just for a naming convenience
            bool is_modified() { return _modified; }

            //! \todo
            bool is_used() { return _mean_vec[0] != 0 || _mean_vec[1] != 0; }

            /** \brief Updates the `_mean_vec`, `_cov_mat`, `_det_cov_mat` and `_inv_cov_mat` if `is_modified` returns true 
              * These calculations have to be done in **this exact order**, because they are all dependent on each other
              * Results in a cached access to every member after recalculation for the current `_observations`
              */
            void update_state()
            {   //std::cerr << "  Updating state, is_modified: " << is_modified() << '\n';
                if (is_modified())
                {   //std::cerr << "  Cone is modified, size / max_N_size: " << _observations.size() << ", " << _apply_variance_step_count << " \n";
                    _update_mean_vec();
                    _update_cov_mat();
                    if (_observations.size() < _apply_variance_step_count) // _apply_variance_step_count)
                    {    
                         // std::cerr << "Before:\n";
                         // std::cerr << "_cov_mat[0]: " << _cov_mat[0] << '\n';
                         // std::cerr << "_cov_mat[3]: " << _cov_mat[3] << '\n';
                        _cov_mat[0] += _variance_xx; // increase the variance radius to allow for a better "association"
                        _cov_mat[3] += _variance_yy; // increase the variance radius to allow for a better "association"
                         // std::cerr << "After:\n";
                         // std::cerr << "_cov_mat[0]: " << _cov_mat[0] << '\n';
                         // std::cerr << "_cov_mat[3]: " << _cov_mat[3] << '\n';
                    }
                    _update_det_cov_mat();
                    _update_inv_cov_mat();
                    _set_modified(false);
                }
            }

            //! Helper function for currying, calls pdf(T x, T y)
            double pdf(std::tuple<T, T, T, T> tup)
            {
                return pdf(std::get<0>(tup), std::get<1>(tup));
            }

            /** \brief Probability density function of this multivarianceiate gaussian
              * Linearized gaussian multiplication for 2x2 matrizes of the mahalanobis distance in the exponent of `e`
              *
              * \todo make a nice image for the explanation
              *
              * **Invariant:** All the members are updated
              */
            double pdf(T x, T y)
            {  
                update_state();
                const double fraction = 1 / std::sqrt(std::pow(2*M_PI, 2) * _det_cov_mat);

                const T xx = _inv_cov_mat[0];
                const T xy = _inv_cov_mat[1];
                const T yy = _inv_cov_mat[3];
                const T x_mean = _mean_vec[0];
                const T y_mean = _mean_vec[1];
                const T x_norm = x - x_mean;
                const T y_norm = y - y_mean;

                const double mahalanobis = 2*(x_norm*y_norm*xy) + std::pow(x_norm,2) * xx + std::pow(y_norm,2) * yy;
                const double factor   = std::exp(-0.5 * mahalanobis);

                return fraction * factor;
            }

            //! Needed to calculate the weighting parameter for this pdf in accordance to all other weights of clusters. Returing double, because we need to divide later
            double get_observations_size(){ return static_cast<double>(_observations.size()); }

            //! Helper function for currying, calls distance_greater_than(T x, T y, T epsilon)
            constexpr bool distance_greater_than(const std::tuple<T, T, T, T> tup, const T epsilon) const
            {
                return distance_greater_than(std::get<0>(tup), std::get<1>(tup), epsilon);
            }

            //! euclidian distance to the cluster mid point
            constexpr T distance(const std::tuple<T, T, T, T> tup) const
            {
                return distance(std::get<0>(tup), std::get<1>(tup));
            }

            //! euclidian distance to the cluster mid point
            constexpr T distance(const std::tuple<T, T> tup) const
            {
                return distance(std::get<0>(tup), std::get<1>(tup));
            }

            //! euclidian distance to the cluster mid point
            constexpr T distance(const T x, const T y) const
            {
                const T x_dist = std::pow(x - _mean_vec[0], 2);
                const T y_dist = std::pow(y - _mean_vec[1], 2);
                return std::sqrt(x_dist + y_dist);
            }

            //! checks if the distances is greater than epsilon from the mean position of the cluster _mean_vec
            constexpr bool distance_greater_than(const T x, const T y, const T epsilon) const
            {
                const T dist = distance(x, y);
                //std::cerr << "    distance_intrinsics: " << res << " => " << (std::abs(epsilon) < res) << "\n";
                return std::abs(epsilon) < dist; 
            }

            //! returns the cluster middle, calculated by `_update_mean_vec()`
            constexpr std::tuple<T, T> get_approx_position() const
            { 
                return { _mean_vec[0], _mean_vec[1] };
            }

            /** \brief Calcualted the relative positional difference of the last 2 observations
              * **Important:** only callable if we have 2 or more _observations
              */
            constexpr std::tuple<T, T> get_relative_pos_difference() const
            {
                const coords_t & o_2 = _observations.back();
                const coords_t & o_1 = *(_observations.end() - 2);
                std::cerr << "        > get_rel_pos_diff():\n" \
                          << "          mean_vec:" << _mean_vec[0] << ", " << _mean_vec[1] << '\n'
                          << "          o_2:"      << o_2[0]       << ", " << o_2[1]       << ", " << o_2[2] << ", " << o_2[3] << '\n'
                          << "          o_1:"      << o_1[0]       << ", " << o_1[1]       << ", " << o_1[2] << ", " << o_1[3] << '\n';
                return std::make_tuple(o_1[2] - o_2[2], o_1[3] - o_2[3]);
            }

        // methods
        private:
            /** \brief setter for our modification flag, this is used to show if we got new information about our position
              * Any access to `_mean_vec`, `_cov_mat`, `_det_cov_mat` and `_inv_cov_mat` triggers their recomputation via `update_state()`
              */
            void _set_modified(bool b){ _modified = b; }

            /** \brief This acces to `_inv_cov_mat` can trigger a recalculation of the cone state. *Cached*
              *
              */ 
            const std::array<T, 4> & _get_inv_cov_mat()
            {
                if (is_modified()) {
                    update_state();
                }
                return _inv_cov_mat;
            }

            /** \brief This access to `_det_cov_mat` can trigger a recalculation of the cone state. *Cached*
              */
            const T & _get_det_cov_mat()
            {
                if (is_modified()) {
                    update_state();
                }
                return _det_cov_mat;
            }

            /** \brief Inverse matrix calculation of the 2x2 `_cov_mat`, saved in `_inv_cov_mat`
              * **Invarianceiant:**
              * * `_det_cov_mat` and `_cov_mat` is up to date
              * * `_det_cov_mat` != 0, this is taken care of in `_update_det_cov_mat()`
              */
            void _update_inv_cov_mat()
            {
                const T a = _cov_mat[0];
                const T b = _cov_mat[1];
                const T c = _cov_mat[2];
                const T d = _cov_mat[3];

                _inv_cov_mat[0] =  d / _det_cov_mat;
                _inv_cov_mat[1] = -b / _det_cov_mat;
                _inv_cov_mat[2] = -c / _det_cov_mat;
                _inv_cov_mat[3] =  a / _det_cov_mat;
            }

            /** \brief Cached determinant calculation of a 2x2 matrix `_cov_mat`, saved in `_det_cov_mat`
              * Property: `_det_cov_mat` will never be 0, because the gaussian sampling in the EM algorithm is not defined for a zero covarianceiance in all dimensions. We then set it to `1`
              * **Invarianceiant:** `_cov_mat` is up to date
              */
            void _update_det_cov_mat()
            {
                const T a = _cov_mat[0];
                const T b = _cov_mat[1];
                const T c = _cov_mat[2];
                const T d = _cov_mat[3];
                _det_cov_mat = a * d - c * b;
            }

            /** \brief Updates the `_mean_vec` with the current `_observations`
              * **Invarianceiant:** `_observations` is up to date
              *
              * \todo parallelize this loop and test the performance time
              */
            void _update_mean_vec()
            {
                const coords_t sums = 
                    std::accumulate(_observations.begin(), _observations.end()
                                  , coords_t { { 0, 0 } }
                                  , [](coords_t acc, coords_t _o)
                    {
                        const T x =  _o[0];
                        const T y =  _o[1];
                        acc[0] += x;
                        acc[1] += y;
                        return acc;
                    });

                const T      x_sum              = sums[0];
                const T      y_sum              = sums[1];
                const size_t observations_count = _observations.size();

                _mean_vec[0] = x_sum / observations_count;
                _mean_vec[1] = y_sum / observations_count;
            }

            /** \brief Updates the `_cov_mat` with the current `_observations` and `_mean_vec`
              * **Invarianceiant:** `_mean_vec`, `_observations` is up to date
              *
              * \todo parallelize this loop and test the performance time
              */
            void _update_cov_mat()
            {
                const T      mean_x             = _mean_vec[0];
                const T      mean_y             = _mean_vec[1];
                const size_t observations_count = _observations.size();
                
                const std::array<T, 3> cov_sum =
                    std::accumulate(_observations.begin(), _observations.end()
                                  , std::array<T,3> { { 0, 0, 0 } }
                                  , [&](std::array<T, 3> & cov_sum, const coords_t & _o)
                    {
                        const T          x = _o[0];
                        const T          y = _o[1];
                        const T   x_factor = x - mean_x;
                        const T   y_factor = y - mean_y;
                        const T xx_summand = std::pow(x_factor, 2);
                        const T yy_summand = std::pow(y_factor, 2);
                        const T xy_summand = x_factor * y_factor;

                        cov_sum[0] += xx_summand;
                        cov_sum[1] += yy_summand;
                        cov_sum[2] += xy_summand;
                        return cov_sum;
                    });

                const T cov_xx = cov_sum[0] / observations_count;
                const T cov_yy = cov_sum[1] / observations_count;
                const T cov_xy = cov_sum[2] / observations_count;

                _cov_mat[0] = cov_xx;
                _cov_mat[1] = cov_xy;
                _cov_mat[2] = cov_xy;
                _cov_mat[3] = cov_yy;
            }

        // member (were private, but for logging purposes we mage them public)
        public:
            //! modification flag, if this is true, we need to recompute `_cov_mat`, `_inv_cov_mat` and `_mean_vec`. `add_observation()` triggers this
            bool _modified;
            /** \brief List of the observations of this particular cone, `add_observation()` can add elements to this list 
              * `_observations` is preallocated in the constructor to a certain size
              */
            std::vector<coords_t> _observations;
            /** \brief Linearized representation of a 2x2 matrix
              * \code
              * a = _cov_mat[0];
              * b = _cov_mat[1];
              * c = _cov_mat[2];
              * d = _cov_mat[3];
              * \endcode
              *
              * ```
              * [ [ a, b ]
              *  ,[ c, d ] ]
              * ``` 
              */
            std::array<T, 4> _cov_mat;
            /** \brief Linearized representation of a 2x2 matrix
              * \code
              * a = _inv_cov_mat[0];
              * b = _inv_cov_mat[1];
              * c = _inv_cov_mat[2];
              * d = _inv_cov_mat[3];
              * \endcode
              *
              * ```
              * [ [ a, b ]
              *  ,[ c, d ] ]
              * ``` 
              */
            std::array<T, 4> _inv_cov_mat;
            //! Determinand of _cov_mat
            T                _det_cov_mat;
            /** \brief mean vector of x and y (0 and 1 index respectivly)
              * \code
              * x = _mean_vec[0];
              * y = _mean_vec[1];
              * \endcode
              */
            std::array<T, 2> _mean_vec;
            //! if set in special constructor, we apply variance to the _cov_mat for xx (index 0)
            T _variance_xx;
            //! if set in special constructor, we apply variance to the _cov_mat for yy (index 3)
            T _variance_yy;
            //! if set in special constructor, we apply _variance_xx and _variance_yy to the _cov_mat until we have seen at least _apply_variance_step_count observations
            size_t _apply_variance_step_count;
    };
} // namespace clara

/*! @} End of Doxygen Groups*/
#endif // CONE_STATE_H
