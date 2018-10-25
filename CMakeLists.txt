# Copyright 2018 municHMotorsport e.V. <info@munichmotorsport.de>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(PROJECT_NAME CLARA)
set(LIB_NAME  clara)
set(EXEC_NAME clara_exec)
set(TEST_NAME clara_test)

project (${PROJECT_NAME})
cmake_minimum_required (VERSION 3.5.1)

## this flag is used to set the compiler flags and should be used in the code
## with #if DEBUG_MODE <code> #else <code> #endif OR the debug print macro
OPTION(DEBUG_MODE "Enables the DEBUG messages and -O0 flags" 1)

OPTION(WITH_TESTS "Enables the compilation of tests" 0)

# set compiler flags
set(CMAKE_CXX_COMPILER=g++)

## Compile as C++14
add_compile_options(-std=c++14)

# Compile differently if DEBUG_MODE is active
if(DEBUG_MODE)
    message("> DEBUG_MODE is ON - no optimizations enabled")
    add_definitions(-DDEBUG_MODE=1)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -pthread -Wall -Wextra         -Wfatal-errors -pedantic -g -fPIC -Wno-reorder")
else()
    message("> DEBUG_MODE is OFF - O2 optimizations enabled")
    add_definitions(-DDEBUG_MODE=0)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -pthread -Wall -Wextra -Werror -Wfatal-errors -pedantic -fPIC -Wno-reorder")
endif()

find_package(kafi 1.0 REQUIRED)
find_package(connector 1.0 REQUIRED)

# add the actual code
add_subdirectory (library)

if(WITH_TESTS)
    message("> Compiling tests for CLARA")
    add_subdirectory (tests)
else()
    message("> NOT compiling tests for CLARA")
endif()
