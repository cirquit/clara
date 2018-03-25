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

#include <blaze/Math.h>
#include "data_association.h"

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
        // /**
        //  * \brief Static allocated identity matrix specialized for doubles
        //  *
        //  * Template arguments:
        //  * * `N`  = Number of rows and columns
        //  * * `SO` = Storage order, e.g `blaze::rowMajor`
        //  * 
        //  * \todo make generic with SFINAE and `is_integral` check on inner type `T` (currently `double`) 
        //  */ 
        // template< size_t   N   
        //         , bool     SO >
        // constexpr blaze::StaticMatrix<double, N, N, SO> create_identity()
        // {
        //     blaze::StaticMatrix<double, N, N, SO> matrix(0);
        //     for (size_t row = 0UL; row < matrix.rows(); ++row)
        //     {
        //         for (size_t col = 0UL; col < matrix.columns(); ++col)
        //         {
        //             if (row == col) matrix(row, col) = 1;
        //         }
        //     }
        //     return matrix;
        // }



        // /*!
        //  * \brief Identity function which takes the `in(0,0)` element and broadcasts it to the `mx1_vector`
        //  * in every dimension
        //  *
        //  * Example:
        //  * * `in<4,1> = [1,2,3,4]`
        //  * * `out<2,1> = [1, 1]`
        //  *
        //  * Template arguments:
        //  * * `N` = state dimensions
        //  * * `M` = sensor dimensions
        //  *
        //  * Return type:
        //  * * std::function( `nx1_vector`, `mx1_vector` ) -> *void*
        //  *
        //  * See more examples in [tests/util_tests.cc](../../tests/util_tests.cc)
        //  */
        // template< size_t N
        //         , size_t M >
        // std::function<void(blaze::StaticMatrix<double, N, 1UL, blaze::rowMajor> &    // aka nx1_vector
        //                  , blaze::StaticMatrix<double, M, 1UL, blaze::rowMajor> &)>  // aka mx1_vector
        // identity_broadcast_function()
        // {
        //     using nx1_vector = typename kafi::jacobian_function<N,M>::nx1_vector;
        //     using mx1_vector = typename kafi::jacobian_function<N,M>::mx1_vector;

        //     const auto f = [&](nx1_vector & input, mx1_vector & output)
        //     {
        //         for (size_t m = 0; m < M; ++m)
        //         {
        //             output(m, 0) = input(0,0);
        //         }
        //     };

        //     return f;
        // }

        // /*!
        //  * \brief Derivative function of util::identity_broadcast_function
        //  *
        //  * Used to create a partial derivative, parametrizable with the return type, mostly 0 or 1
        //  *
        //  * Template arguments:
        //  * * `N` = state dimensions
        //  *
        //  * Return type:
        //  * * std::function( `const nx1_vector` ) -> *double*
        //  *
        //  * See examples in [tests/util_tests.cc](../../tests/util_tests.cc)
        //  */
        // template< size_t N >
        // const std::function<double(const blaze::StaticMatrix<double, N, 1UL, blaze::rowMajor> &)> // aka par_jacobi_func
        // identity_derivative(double ret)
        // {
        //     using nx1_vector      = typename kafi::jacobian_function<N,1UL>::nx1_vector;
        //     using par_jacobi_func = typename kafi::jacobian_function<N,1UL>::par_jacobi_func;

        //     const par_jacobi_func f_ = [ret](const nx1_vector & in){
        //         UNUSED(in);
        //         return ret;
        //     };
        //     return f_;
        // }
        
        // /*! \brief Helper function to create a broadcasting identity function for easier tests on multiple dimensions
        //  * 
        //  * Template arguments:
        //  * * `N`  = state dimensions
        //  * * `M`  = sensor dimensions
        //  *
        //  * See examples in [tests/jacobian_function_tests.cc](../../tests/jacobian_function_tests.cc)
        //  *
        //  * Example with `N = 3` and `M = 5`:
        //  * ```
        //  *
        //  *    _N_        __M ____    
        //  *   /   \      /        \
        //  * f[x,y,z] =  [x,x,x,x,x]    (these vectors are actually transposed)
        //  *              0 1 2 3 4
        //  *
        //  * ```
        //  * Formula:
        //  * ```
        //  *
        //  *              ______N_(3)_____
        //  *             /                \ __
        //  * F[x,y,z] = [[0/dx, 0/dy, 0/dz]   \
        //  *             [1/dx, 1/dy, 1/dz]   |
        //  *             [2/dx, 2/dy, 2/dz]]  M (5)
        //  *             [3/dx, 3/dy, 3/dz]]  |
        //  *             [4/dx, 4/dy, 4/dz]]__/
        //  *
        //  * ```
        //  * Calculation for our broadcast identity function - util::identity_broadcast_function
        //  * ```
        //  *
        //  * F[x,y,z] = [[ 1, 0, 0 ]
        //  *             [ 1, 0, 0 ]
        //  *             [ 1, 0, 0 ]
        //  *             [ 1, 0, 0 ]
        //  *             [ 1, 0, 0 ]]
        //  *
        //  * ```
        //  */
        // template< size_t N
        //         , size_t M >
        // jacobian_function<N,M> create_identity_jacobian()
        // {
        //     using func            = typename kafi::jacobian_function<N,M>::func;
        //     using par_jacobi_func = typename kafi::jacobian_function<N,M>::par_jacobi_func;
        //     using jacobi_func     = typename kafi::jacobian_function<N,M>::jacobi_func;

        //           func            f      = identity_broadcast_function<N,M>();
        //     const par_jacobi_func f_one  = identity_derivative<N>(1);
        //     const par_jacobi_func f_zero = identity_derivative<N>(0);
        //     jacobi_func F(f_zero);

        //     for (size_t row = 0UL; row < M; ++row)
        //     {
        //         F(row, 0) = f_one;
        //     }
        //     return jacobian_function<N, M>(f, F);
        // }
    } // namespace util
} // namespace kafi
/*! @} End of Doxygen Groups*/
#endif // CLARA_UTIL_H