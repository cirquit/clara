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
 *  * **yaw angle** to the car direction. This is in the range of \todo ask sheldon for yaw angle
 * 
 *  Cones are saved by their absolute position (x,y), based on the starting point of the car (usually `(0,0)`)
 * 
 *  Template arguments:
 *  * `N` = maximum amount of cones which we'll be able to identify
 */ 
template<size_t N> 
class data_association
{

    //! `x` and `y` position
    using cone_data = std::tuple<double, double>;

    // constructors
    public:
        data_association();
    // member
    private: 

    // member
    public: 
        std::array<cone_data, N> cone_states;
        // std::vector<

};

} // namespace clara

/*! @} End of Doxygen Groups*/
#endif // DATA_ASSOCIATION_H