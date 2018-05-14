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

#ifndef CLARA_UTIL_H
#define CLARA_UTIL_H

/*!
 *  \addtogroup clara::util
 *  @{
 */
#include <iostream>
#include <algorithm>
#include <blaze/Math.h>
#include <functional>
#include <ctime>

//! unused macro to avoid errors because of nonuse of declared variables
#define UNUSED(x) (void)(x)

namespace clara
{   /** \brief Utilities namespace so summarize often used functions from different modules
      *
      */
    namespace util {

        //! Converts radians to angles. Does not do `modulo 360°`
        constexpr double rad_to_angle(const double radian)
        {
          return radian * (180.0 / M_PI);
        }

        //! Converts angles to radians. Does not do `modulo 6.2830...`
        constexpr double angle_to_rad(const double angle)
        {
          return angle * (M_PI / 180.0);
        }

        //! Specialized advance_all for a single iterator which is incremented
        template <typename Iterator>
        void advance_all(Iterator & iterator) {
          ++iterator;
        }

        //! Increments all iterators by one. Defined for any amount of iterators
        template <typename Iterator, typename ... Iterators>
        void advance_all(Iterator  & iterator
                       , Iterators & ... iterators) {
            ++iterator;
            advance_all(iterators...);
        }

        /** \brief Iterates over all iterators with a function that appends its result to OutputIt
         *  Iterators have to have the **same size**!
         *  Example:
         *
         *  \code
         *  std::vector<int> xs { 1, 2, 3 };
         *  std::vector<int> ys { 2, 4, 8 };
         *  std::vector<int> res { 0, 0, 0 };
         * 
         *  zipWith([](int x, int y){ return x+y; }),
         *             xs.begin(),
         *             xs.end(),
         *             res.begin(),
         *             ys.begin());
         * 
         * // res = { 3, 6, 11 };
         * \endcode
         */ 
        template <typename Function, typename InputIt, typename OutputIt, typename ... Iterators>
        OutputIt zipWith(Function func,
                         InputIt first,
                         InputIt last,
                         OutputIt d_first,
                         Iterators ... iterators)
        {
            for (; first != last; ++first, advance_all(iterators...))
              *d_first++ = func(*first, *(iterators)...);
            return d_first;
        }
        
        /** \brief Iterates over all iterators with a function that returns void
         *  Iterators have to have the **same size**!
         *  Example:
         *
         *  \code
         *  std::vector<int> xs { 1, 2, 3 };
         *  std::vector<int> ys { 2, 4, 8 };
         *  std::vector<int> res { 0, 0, 0 };
         * 
         *  zipWith_([](int x, int y){ std::cout << x << y << '\n'; }),
         *           xs.begin(),
         *           xs.end(),
         *           ys.begin());
         * \endcode
         */ 
        template <typename Function, typename InputIt, typename ... Iterators>
        void zipWith_(Function func,
                         InputIt first,
                         InputIt last,
                         Iterators ... iterators)
        {
            for (; first != last; ++first, advance_all(iterators...))
                func(*first, *(iterators)...);
        }

        /** \brief Adds an index to every element in an iterators
          *
          * Source: blatantly copied from http://stackoverflow.com/questions/3752019/how-to-get-the-index-of-a-value-in-a-vector-using-for-each
          */ 
        template <typename InputT, typename Function>
        Function enumerate(InputT first,
                           InputT last,
                           typename std::iterator_traits<InputT>::difference_type initial,
                           Function func)
        {
            for (; first != last; ++first, ++initial)
                func(initial, *first);
            return func;
        }

        //! Handy shortcut for simple loops
        template <class Iterable, typename Function>
        void for_each_(Iterable & container, Function func)
        {
            std::for_each(container.begin(), container.end(), func);
        }

        //! Handy shortcut for simple loops
        template <class Iterable, typename Function>
        void transform_(Iterable & container, Function func)
        {
            std::transform(container.begin(), container.end(), container.begin(), func);
        }

        template<class T>
        T timeit(std::function<T> f) {
            std::clock_t start = std::clock();
            T res = f();
            std::cerr << "Elapsed time: " << (std::clock() - start) / static_cast<double>(CLOCKS_PER_SEC / 1000) << "ms\n";
            return res;
        }

        //! Handy shortcut for "is the element in this container"
        template<class Iterable, class Element>
        bool contains(Iterable & container, Element & elem)
        {
            return std::find(container.begin(), container.end(), elem) != container.end();
        }

        //! calculate the mean of a tuple<double, double> container
        template< class Iterable >  
        std::tuple<double, double> mean_accumulate(Iterable container)
        {
            std::tuple<double, double> start = std::make_tuple(0, 0);
            double x_sum, y_sum;
            std::tie(x_sum, y_sum) = std::accumulate(container.begin(), container.end(), start
                        , [](std::tuple<double, double> acc
                            ,std::tuple<double, double> elem)
                        {
                            double acc_x = 0;
                            double acc_y = 0;
                            double elem_x = 0;
                            double elem_y = 0;
                            std::tie(acc_x, acc_y) = acc;
                            std::tie(elem_x, elem_y) = elem;
                            return std::make_tuple(acc_x + elem_x, acc_y + elem_y);
                        });
            return std::make_tuple(x_sum / static_cast<double>(container.size())
                                  ,y_sum / static_cast<double>(container.size()) );
        }

        //! prints the cluster to the std::cout as a valid python library
        // template< class T >
        // void print_data_assoc(clara::data_association<T> & da, int color)
        // {   
        //     const std::vector<clara::cone_state<T>> & cluster = da.get_cluster();
        //     const double cluster_weight = 0; // deprecated, numpy needs this value to have a valid matrix \todo fix python script

        //     if (color == 0) { std::cout << "yellow_cone_data = np.array([\n"; }
        //     if (color == 1) { std::cout << "blue_cone_data = np.array([\n"; }
        //     if (color == 2) { std::cout << "red_cone_data = np.array([\n"; }
        //     clara::util::for_each_(cluster, [&](const clara::cone_state<double> & cs)
        //     {
        //         double mean_x = cs._mean_vec[0];
        //         double mean_y = cs._mean_vec[1];
        //         double cov_xx = cs._cov_mat[0];// - 0.45;
        //         double cov_xy = cs._cov_mat[1];
        //         double cov_yy = cs._cov_mat[3];// - 0.45;

        //         std::string np_b = "np.array([";
        //         std::string np_e = "])";
        //         if ( mean_x != 0 || mean_y != 0 ) {
        //             std::cout << np_b
        //                       << "[" << mean_x << ", " << mean_y << "]"  << ", "
        //                       << "[[" << cov_xx << ", " << cov_xy << "]" << ", "
        //                       << "[" << cov_xy << ", " << cov_yy << "]]" << ", "
        //                       << "["  << cluster_weight << "]"
        //                       << np_e << ",\n"; 
        //         }
        //     });
        //     std::cout << "]);\n";
        // }

        // //! prints the observations to the std::cout as a valid python library
        // template < class T >
        // void print_observations(clara::data_association<T> & da, int color)
        // {
        //     const std::vector<clara::cone_state<T>> & cluster = da.get_cluster();

        //     if (color == 0) { std::cout << "yellow_obs_cone_data = np.array([\n"; }
        //     if (color == 1) { std::cout << "blue_obs_cone_data = np.array([\n"; }
        //     if (color == 2) { std::cout << "red_obs_cone_data = np.array([\n"; }
        //     clara::util::for_each_(cluster, [&](const clara::cone_state<double> & cs)
        //     {
        //         double mean_x = cs._mean_vec[0];
        //         double mean_y = cs._mean_vec[1];

        //         if ( mean_x != 0 || mean_y != 0 ) {
        //             for(auto obs : cs._observations)
        //             {
        //                 std::cout << "[" << obs[0] << ", " << obs[1] << "],\n"; 
        //             }
        //         }
        //     });
        //     std::cout << "]);\n";
        // }

        //! equal with an epsilon bound, explicitly casting any type to double
        template< class T >
        bool approx(const T & a, const T & b, const double epsilon)
        {
            const double _a = static_cast<double>(a);
            const double _b = static_cast<double>(b);
            const double _eps = std::abs(epsilon);

            const double small_bounds = _b - _eps;
            const double high_bounds  = _b + _eps;
            return _a >= small_bounds && _a <= high_bounds;
        }

        //! distance, defined between 0 and MAX_DOUBLE
        template< class T >
        double distance(const T & a, const T & b)
        {
            return std::abs(static_cast<double>(a) - static_cast<double>(b));
        }

        //! euclidean distance, casting every type to double
        template< class T >
        double euclidean_distance(const std::tuple<T, T> & a
                                , const std::tuple<T, T> & b)
        {
            const double & x_old = static_cast<double>(std::get<0>(a));
            const double & y_old = static_cast<double>(std::get<1>(a));
            const double & x_new = static_cast<double>(std::get<0>(b));
            const double & y_new = static_cast<double>(std::get<1>(b));
            return std::sqrt(std::pow(x_old - x_new, 2) +  std::pow(y_old - y_new, 2));
        }

    } // namespace util
} // namespace clara
/*! @} End of Doxygen Groups*/
#endif // CLARA_UTIL_H