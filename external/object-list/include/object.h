// Copyright 2017 municHMotorsport e.V. <info@munichmotorsport.de>
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

#ifndef OBJECT_SRC_OBJECT_H_
#define OBJECT_SRC_OBJECT_H_
#define MAX_LISTSIZE 30

#include <stdint.h>

/** 
 *
 * Member:
 *    * `double`: x_car X   coordinate of the car in [m]
 *    * `double`: y_car     coordinate of the car in [m]
 *    * `double`: angle_yaw Azimute angle of the car in [rad]
 *    * `double`: vx velocity of the car in x-direction in [m/s]
 *    * `double`: vy velocity of the car in y-direction in [m/s]
 *    * `double`: ax acceleration of the car in x-direction in [m/s^2]
 *    * `double`: ay acceleration of the car in y-direction in [m/s^2]
 *    * `double`: yaw_rate of the car in [rad/s]
 *    * `double`: distance to from the camera to the cone in [m]
 *    * `double`: angle to the cone in driving direction in [rad]
 *    * `double`: elapsed time from the start of the simulation in [s]
 */
typedef struct { 
    double x_car;
    double y_car;
    double angle_yaw;
    double vx;
    double vy;
    double ax;
    double ay;
    double yaw_rate;
    double distance; 
    double angle; 
    double time_s;
    double steering_rad;
    int type;
} object_t;


/*
 * \brief The object_list_t struct contains object_t structs within itself
 *
 * Elements:
 *    * `uint32_t`: size is the number of `object_t` elements for being comaptible with windows
 *    * `object_t`: element is an array of `object_t` elements
 */
typedef struct {
    uint32_t  size;
    object_t  element[MAX_LISTSIZE];
    double    time_s;
} object_list_t;


#endif  // OBJECT_SRC_OBJECT_H_

