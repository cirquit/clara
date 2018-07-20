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

#ifndef CLARA_CLOCK_H
#define CLARA_CLOCK_H

#include <chrono>

namespace clara
{
    /** \brief clock based on chrono to get the difference time. 
     *
     */
    class clara_clock_t
    {

    // constructors
    public:
        clara_clock_t(double init_delay_s)
        : _started(false)
        , _init_delay_s(init_delay_s)
        { }

    // methods
    public:

        //! helper function
        void start_if_not_started()
        {
            if (!_started)
            {
                start_clock();
            }
        }

        //! starts the clock to use with get_diff_time_s and uses a custom delay 
        void start_clock()
        {   
            _started = true;
            _start = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(_init_delay_s * 1000)));

        }

        //! get time difference from last call to start_clock in seconds. Restarts the clock
        double get_diff_time_s()
        {
            auto end = std::chrono::high_resolution_clock::now();
            int elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - _start).count();
            start_clock();
            return elapsed_ms / 1000.0;
        }

        //! checks if the clock is already running
        bool started() { return _started; };

    // member
    public:
        //! clock to apply velocities from last observed model
        std::chrono::time_point<std::chrono::high_resolution_clock> _start;

    // member
    private:
        //! is the clock already running
        bool _started;
        //! initial delay after starting the clock
        double _init_delay_s;
    };
}

#endif // VEHICLE_STATE
