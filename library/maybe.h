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

#ifndef MAYBE_H
#define MAYBE_H

#include <memory>

/*!
 *  \addtogroup modi-scheduler
 *  @{
 */

/** \brief concept namespace
 *
 */
namespace concept {

    /** \brief Maybe is a concept of a value which has a type encoding of validness
      * 
      * We always have to check the with has_value if there is actually a value saved
      */
    template <class T>
    class maybe {

    // types
    public:
        using value_t = T;
    // constructor
    public:
        //! constructor with value initialization
        maybe(T value)
        : _value(value)
        , _valid(true) 
        { }

        //! constrictor without a value initialization
        maybe()
        : _valid(false)
        { }

    // methods
    public:
        //! checks if we have a value
        bool has_value() { return _valid; }
        //! inverse of has_value()
        bool has_no_value() { return !has_value(); }
        //! unsafe value get. use has_value to insure correctness
        value_t get_value() { return _value; }
        //! set value, set valid = true
        void set_value(value_t value)
        {
            _value = value;
            _valid = true;
        }
    // member
    private:
        value_t _value;
        bool    _valid;
    };
} // namespace concept

#endif // MAYBE_H
