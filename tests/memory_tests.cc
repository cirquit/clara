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

#include "catch.h"

#include "../library/memory.h"

TEST_CASE( "memory.h", "" ) {

    clara::memory_t count_02( 2.0 );

    REQUIRE(count_02.get_mean() == 0.0);

    count_02.add_value(1.0);

    REQUIRE(count_02.get_mean() == 1.0);

    count_02.add_value(3.0);

    REQUIRE(count_02.get_mean() == 2.0);

    count_02.add_value(5.0);

    REQUIRE(count_02.get_mean() == 4.0);

    count_02.reset();

    REQUIRE(count_02.get_mean() == 0.0);

    count_02.add_value(2.5);
    count_02.add_value(5.0);
    REQUIRE(count_02.get_mean() == 3.75);
}

