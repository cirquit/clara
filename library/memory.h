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

#ifndef MEMORY_H
#define MEMORY_H

#include <numeric>
#include <iostream>

namespace clara
{
    class memory_t
    {

    // constructors
    public:

        /** \brief Memory constructor to get the mean of all values, count is the amount of values we want to buffer
          *
          */
        memory_t(unsigned int count)
        : _count( count )
        , _ix( 0 )
        {
            _memory.reserve(_count);
        }

    // methods
    public:

        //! mean of all values we're currently holding, default to zero if no value was added 
        double get_mean()
        {
            if (_memory.size() == 0) return 0.0;
            return std::accumulate(_memory.begin(), _memory.end(), 0.0) / static_cast<double>(_memory.size());
        }

        //! adds the value to the _memory vector
        void add_value(double value)
        {
            if (_memory.size() < _count)
            {
                _memory.emplace_back(value);
            } else {
                _memory[_ix % _count] = value;
                _ix++;
            }
        }

        //! clears the memory
        void reset()
        {
            _memory.clear();
            _ix = 0;
        }

    // member
    public:

    // member
    private:

        std::vector< double > _memory;
        unsigned int _ix;
        unsigned int _count;
    };
}


#endif // VEHICLE_STATE
