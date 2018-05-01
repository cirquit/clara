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
            std::tuple<double, double> start = { 0, 0 };
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

    } // namespace util
} // namespace clara
/*! @} End of Doxygen Groups*/
#endif // CLARA_UTIL_H