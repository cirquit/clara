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
#include <blaze/Math.h>

//! unused macro to avoid errors because of nonuse of declared variables
#define UNUSED(x) (void)(x)

#if !defined(DEBUG_MODE)
    #error DEBUG_MODE has to be defined through 'cmake .. -DDEBUG_MODE=[0/1]'
#endif

#if DEBUG_MODE
//! debug message macro to print out the file, line and the message where something happend 
#define DEBUG_MSG(msg) fprintf(stderr, \
                               "[INFO] (%64s:%4d:%16s) %s\n", \
                               __FILE__, \
                               __LINE__, \
                               __func__, \
                               msg)
#else
//! debug message macro to print out the file, line and the message where something happend. Does **nothing** in DEBUG_MODE 
#define DEBUG_MSG(msg)
#endif // DEBUG_MODE (in [package-name]/CMakeLists.txt)

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

        //! \todo
        template <typename Iterator>
        void advance_all(Iterator & iterator) {
          ++iterator;
        }

        //! \todo
        template <typename Iterator, typename ... Iterators>
        void advance_all(Iterator  & iterator
                       , Iterators & ... iterators) {
            ++iterator;
            advance_all(iterators...);
        }

        //! \todo
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
        
        /** \brief \todo
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

    } // namespace util
} // namespace clara
/*! @} End of Doxygen Groups*/
#endif // CLARA_UTIL_H