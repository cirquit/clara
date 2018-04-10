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
     *  To do EM, we need to calculate the mean und covariance matrix over `x, y`. Because we know we only have a 2-dimensional state, we provide
     *  the needed functionality to do a gauss-sampling. See `get_inv_cov_mat()` and `get_det_cov_mat()`
     *
     *  Because of the assumption that new cones 
     *
     *  Template arguments:
     *  * `T` is the type of the interal coordinates
     *
     *  Restrictions:
     *  * `T` has to be moveable
     *
     * \todo add `is_integral` test for template `T`
     */
    template <typename T>
    class cone_state
    {
        public:
            //! for convenience purposes
            using self_t   = cone_state<T>;
            //! 
            using coords_t = std::array<T,2>;
        // constructors
        public:
            //! default constructor without noisy initialization
            cone_state() : cone_state(0, 0, 0) { }

            //! noisy constructor, we apply the _noise_xx and _noise_yy to the covariance matrix until we have at least  seen _end_of_noise_ob_count observations
            cone_state(T noise_xx, T noise_yy, size_t end_of_noise_ob_count)
            : _modified(false)
            , _cov_mat {}
            , _inv_cov_mat {}
            , _det_cov_mat {}
            , _mean_vec {}
            , _noise_xx(noise_xx)
            , _noise_yy(noise_yy)
            , _end_of_noise_ob_count(end_of_noise_ob_count)
            {   // magic number (10 rounds à 10 times seeing the same cone)
                _observations.reserve(10 * 10);
            }

            //! allow copies
            cone_state(const self_t & other)  = default;

            //! allow moves
            // cone_state(self_t && other) = default;

        // methods
        public:

            //! Helper function for currying, calls add_observation(T x, T y)
            void add_observation(std::tuple<T, T> tup)
            {   
                add_observation(std::get<0>(tup), std::get<1>(tup));
            }

            /** \brief Adds an observation of coordiantes to the cone state
              *
              * This should be called if the EM algorithm determined with a sufficient probability that those coordiantes are in fact associated with this cone
              * For the thoughful people, we use `set_modified()` and don't recalculate the `_cov_mat` and `_mean_vec` because we can have multiple observations of the same cone at the same time
              * This highly depends on the detection algorithm
              * 
              * \todo test emplace_back instead of push_back
              * \todo test streaming mean / covariance functions so we don't need to accumulate all observations
              */
            void add_observation(T x, T y)
            {   
                _observations.push_back( coords_t { { x, y } } );
                set_modified(true);
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
                {   //std::cerr << "  Cone is modified, size / max_N_size: " << _observations.size() << ", " << _end_of_noise_ob_count << " \n";
                    update_mean_vec();
                    update_cov_mat();

                    if (_observations.size() < 4) // _end_of_noise_ob_count)
                    {    //std::cerr << "_cov_mat[0]: " << _cov_mat[0] << '\n';
                         //std::cerr << "_cov_mat[3]: " << _cov_mat[3] << '\n';

                        _cov_mat[0] += 0.25;
                        _cov_mat[3] += 0.25;
                    //     _cov_mat[0] += _noise_xx; // increase the variance radius to allow for a better "association"
                    //     _cov_mat[3] += _noise_yy; // increase the variance radius to allow for a better "association"
                         //std::cerr << "_cov_mat[0]: " << _cov_mat[0] << '\n';
                         //std::cerr << "_cov_mat[3]: " << _cov_mat[3] << '\n';
                    }
                    update_det_cov_mat();
                    update_inv_cov_mat();
                    set_modified(false);
                }
            }

            //! Helper function for currying, calls pdf(T x, T y)
            double pdf(std::tuple<T, T> tup)
            {
                return pdf(std::get<0>(tup), std::get<1>(tup));
            }

            /** \brief Probability density function of this multivariate gaussian
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
                //std::cerr << "fraction: " << fraction << '\n';

                const T xx = _inv_cov_mat[0];
                const T xy = _inv_cov_mat[1];
                const T yy = _inv_cov_mat[3];
                const T x_mean = _mean_vec[0];
                const T y_mean = _mean_vec[1];
                const T x_norm = x - x_mean;
                const T y_norm = y - y_mean;

                const double mahalanobis = 2*(x_norm*y_norm*xy) + std::pow(x_norm,2) * xx + std::pow(y_norm,2) * yy;
                //std::cerr << "mahalanobis: " << mahalanobis << '\n';
                const double factor   = std::exp(-0.5 * mahalanobis);
                //std::cerr << "factor: " << factor << '\n';

                return fraction * factor;
            }

            //! Needed to calculate the weighting parameter for this pdf in accordance to all other weights of clusters. Returing double, because we need to divide later
            double get_observations_size(){ return static_cast<double>(_observations.size()); }

            //! Helper function for currying, calls distance_greater_than(T x, T y, T epsilon)
            constexpr bool distance_greater_than(const std::tuple<T, T> tup, const T epsilon) const
            {
                return distance_greater_than(std::get<0>(tup), std::get<1>(tup), epsilon);
            }

            //! checks if the distances is greater than epsilon from the mean position of the cluster _mean_vec
            constexpr bool distance_greater_than(const T x, const T y, const T epsilon) const
            {
                const T x_dist = std::pow(x - _mean_vec[0], 2);
                const T y_dist = std::pow(y - _mean_vec[1], 2);
                const T res    = std::sqrt(x_dist + y_dist);
                //std::cerr << "    distance_intrinsics: " << res << " => " << (std::abs(epsilon) < res) << "\n";
                return std::abs(epsilon) < res; 
            }

        // methods
        private:
            /** \brief setter for our modification flag, this is used to show if we got new information about our position
              * Any access to `_mean_vec`, `_cov_mat`, `_det_cov_mat` and `_inv_cov_mat` triggers their recomputation via `update_state()`
              */
            void set_modified(bool b){ _modified = b; }


            /** \brief This acces to `_inv_cov_mat` can trigger a recalculation of the cone state. *Cached*
              *
              */ 
            const std::array<T, 4> & get_inv_cov_mat()
            {
                if (is_modified()) {
                    update_state();
                }
                return _inv_cov_mat;
            }

            /** \brief This access to `_det_cov_mat` can trigger a recalculation of the cone state. *Cached*
              */
            const T & get_det_cov_mat()
            {
                if (is_modified()) {
                    update_state();
                }
                return _det_cov_mat;
            }

            /** \brief Inverse matrix calculation of the 2x2 `_cov_mat`, saved in `_inv_cov_mat`
              * **Invariant:**
              * * `_det_cov_mat` and `_cov_mat` is up to date
              * * `_det_cov_mat` != 0, this is taken care of in `update_det_cov_mat()`
              */
            void update_inv_cov_mat()
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
              * Property: `_det_cov_mat` will never be 0, because the gaussian sampling in the EM algorithm is not defined for a zero covariance in all dimensions. We then set it to `1`
              * **Invariant:** `_cov_mat` is up to date
              */
            void update_det_cov_mat()
            {
                const T a = _cov_mat[0];
                const T b = _cov_mat[1];
                const T c = _cov_mat[2];
                const T d = _cov_mat[3];
                _det_cov_mat = a * d - c * b;
                //std::cerr << "cov-mat: " << a << ", " << b << ", " << c << ", " << d << ", " << _det_cov_mat <<'\n';
               // if (std::abs(_det_cov_mat) < 0.0000000000000000000001) {
                // std::cout << "< 0.000000001 triggered\n";
                 // _det_cov_mat = 1; }
            }

            /** \brief Updates the `_mean_vec` with the current `_observations`
              * **Invariant:** `_observations` is up to date
              *
              * \todo parallelize this loop and test the performance time
              */
            void update_mean_vec()
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
              * **Invariant:** `_mean_vec`, `_observations` is up to date
              *
              * \todo parallelize this loop and test the performance time
              */
            void update_cov_mat()
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
              * \todo make the preallocation configurable
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

            //! if set in special constructor, we apply noise to the covariance matrix in for x,x
            T _noise_xx {0};
            //! if set in special constructor, we apply noise to the covariance matrix in for yy
            T _noise_yy {0};
            //! if set in special constructor, we apply _noise_xx and _noise_yy to the covariance matrix until we have seen at least _end_of_noise_ob_count observations
            size_t _end_of_noise_ob_count {0};

    };
} // namespace clara

/*! @} End of Doxygen Groups*/
#endif // DATA_ASSOCIATION_H
