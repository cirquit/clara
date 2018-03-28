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

#include <blaze/Math.h>
#include <vector>
#include <iostream>
#include <random>
#include "catch.h"

#include "../library/util.h"

TEST_CASE("util.h:angle-radians convertion", "[rad]") {

  REQUIRE(clara::util::angle_to_rad(0) == 0.0);
  REQUIRE(clara::util::angle_to_rad(90) == Approx(1.5708));
  REQUIRE(clara::util::angle_to_rad(180) == Approx(3.1415926));
  REQUIRE(clara::util::angle_to_rad(-90) == Approx(-1.5708));

  REQUIRE(clara::util::rad_to_angle(0) == 0.0);
  REQUIRE(clara::util::rad_to_angle(1.5708) == Approx(90.0));
  REQUIRE(clara::util::rad_to_angle(3.1415926) == Approx(180.0));
  REQUIRE(clara::util::rad_to_angle(-1.5708) == Approx(-90));

  for(float f = 0.0; f < 360.1; f += 0.1)
  {
    REQUIRE(clara::util::rad_to_angle(clara::util::angle_to_rad(f)) == Approx(f));
  }
}
