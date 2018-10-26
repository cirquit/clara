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
set(UNIQUE_DEBUG_ID CLARA) 
set(CPP_LIB_NAME  clara)
set(EXEC_NAME clara_exec)
set(TEST_NAME clara_test)


project (${PROJECT_NAME})
cmake_minimum_required (VERSION 3.5.1)

##########################
## command line options
##########################

# a flag to enable tests 
OPTION(ENABLE_TESTS_${UNIQUE_DEBUG_ID} "Enables the compilation of tests (default: on)" ON)

# sets the debug level
set(DEBUG_LEVEL_${UNIQUE_DEBUG_ID} "2" CACHE STRING "Sets the DEBUG Level (default: 2):
                            \     * 0 ~ debugging disabled \n
                            \     * 1 ~ only critical debugging messages are enabled \n
                            \     * 2 ~ every debugging message is enabled \n")
# 
OPTION(ENABLE_OPTIMIZATIONS_${UNIQUE_DEBUG_ID} "Enables the optimization of the compiler (default: off)" OFF)

# allow colorized output
if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

##########################
## compiler options
##########################

# set default compiler (I use 'sudo update-alternatives --config=gcc/g++') to switch the global compiler links
set(CMAKE_C_COMPILER=gcc)
set(CMAKE_CXX_COMPILER=g++)

## compile with the C++ 2014 standart
add_compile_options(-std=c++14)

# set the compiler flags according to the ENABLE_OPTIMIZATIONS flag
if(ENABLE_OPTIMIZATIONS_${UNIQUE_DEBUG_ID})
    message(STATUS "${BoldWhite}${PROJECT_NAME}${ColourReset}: ${BoldGreen}Enabled${ColourReset} optimizations: -O2, change with ${BoldWhite}-DENABLE_OPTIMIZATIONS_${UNIQUE_DEBUG_ID}=OFF${ColourReset}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
else()
    message(STATUS "${BoldWhite}${PROJECT_NAME}${ColourReset}: ${BoldRed}Disabled${ColourReset} optimizations: -O0, change with ${BoldWhite}-DENABLE_OPTIMIZATIONS_${UNIQUE_DEBUG_ID}=ON${ColourReset}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
endif()

# add the corresponding flags to the compiler to allow the macro generation
if (DEBUG_LEVEL_${UNIQUE_DEBUG_ID} STREQUAL "0")
    message(STATUS "${BoldWhite}${PROJECT_NAME}${ColourReset}: DEBUG_LEVEL_${UNIQUE_DEBUG_ID} is set to 0, debugging is ${BoldRed}disabled${ColourReset} with the flag ${BoldWhite}-DDEBUG_LEVEL_${UNIQUE_DEBUG_ID}=0${ColourReset}")
    add_definitions(-DDEBUG_LEVEL_${UNIQUE_DEBUG_ID}_0)
    # create a header for automatic debugging
    file(WRITE "library/autogen-${UNIQUE_DEBUG_ID}-macros.h"
"
\#ifndef AUTOGEN_${UNIQUE_DEBUG_ID}_MACROS_H
\#define AUTOGEN_${UNIQUE_DEBUG_ID}_MACROS_H
\    #include <string.h>
\    //! macro preprocessing of the file standart to get the filename instead of the full filepath
\    #define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

\    // if debugging is disabled, the macros don't do anything 

\    //! logging based on the DEBUG_LEVEL_${UNIQUE_DEBUG_ID} defined while building with cmake (only active with DEBUG_LEVEL_${UNIQUE_DEBUG_ID}=2)
\    #define DEBUG_MSG_${UNIQUE_DEBUG_ID}(msg)
\    //! logging based on the DEBUG_LEVEL_${UNIQUE_DEBUG_ID} defined while building with cmake (active with DEBUG_LEVEL_${UNIQUE_DEBUG_ID}=1 and 2)
\    #define DEBUG_CRIT_MSG_${UNIQUE_DEBUG_ID}(msg)
\#endif // AUTOGEN_${UNIQUE_DEBUG_ID}_MACROS_H")

elseif( DEBUG_LEVEL_${UNIQUE_DEBUG_ID} STREQUAL "1" )
    message(STATUS "${BoldWhite}${PROJECT_NAME}${ColourReset}: DEBUG_LEVEL_${UNIQUE_DEBUG_ID} is set to 1, ${BoldRed}critical${ColourReset} debugging is ${BoldGreen}enabled${ColourReset} with the flag ${BoldWhite}-DDEBUG_LEVEL_${UNIQUE_DEBUG_ID}=1${ColourReset}")
    add_definitions(-DDEBUG_LEVEL_${UNIQUE_DEBUG_ID}_1)
    # create a header for automatic debugging
    file(WRITE "library/autogen-${UNIQUE_DEBUG_ID}-macros.h"
"
\#ifndef AUTOGEN_${UNIQUE_DEBUG_ID}_MACROS_H
\#define AUTOGEN_${UNIQUE_DEBUG_ID}_MACROS_H
\    #include <string.h>
\    //! macro preprocessing of the file standart to get the filename instead of the full filepath
\    #define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

\    #include <iostream>
\    //! logging based on the DEBUG_LEVEL_${UNIQUE_DEBUG_ID} defined while building with cmake (only active with DEBUG_LEVEL_${UNIQUE_DEBUG_ID}=2)
\    #define DEBUG_MSG_${UNIQUE_DEBUG_ID}(msg)
\    //! logging based on the DEBUG_LEVEL_${UNIQUE_DEBUG_ID} defined while building with cmake (active with DEBUG_LEVEL_${UNIQUE_DEBUG_ID}=1 and 2)
\    #define DEBUG_CRIT_MSG_CT(msg) std::cerr << \"[${UNIQUE_DEBUG_ID} - \" \\
\                        << __FILENAME__ << ':' \\
\                        << __LINE__ << ':'     \\
\                        << __func__ << \"()]: \"    \\
\                        << msg;
\#endif // AUTOGEN_${UNIQUE_DEBUG_ID}_MACROS_H")
elseif( DEBUG_LEVEL_${UNIQUE_DEBUG_ID} STREQUAL "2" )
    message(STATUS "${BoldWhite}${PROJECT_NAME}${ColourReset}: DEBUG_LEVEL_${UNIQUE_DEBUG_ID} is set to 2, ${BoldGreen}all debugging is enabled${ColourReset} with the flag ${BoldWhite}-DDEBUG_LEVEL_${UNIQUE_DEBUG_ID}=2${ColourReset}")
    add_definitions(-DDEBUG_LEVEL_${UNIQUE_DEBUG_ID}_2)
    # create a header for automatic debugging
    file(WRITE "library/autogen-${UNIQUE_DEBUG_ID}-macros.h"
"
\#ifndef AUTOGEN_${UNIQUE_DEBUG_ID}_MACROS_H
\#define AUTOGEN_${UNIQUE_DEBUG_ID}_MACROS_H
\    #include <string.h>
\    //! macro preprocessing of the file standart to get the filename instead of the full filepath
\    #define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

\    #include <iostream>
\    //! logging based on the DEBUG_LEVEL_${UNIQUE_DEBUG_ID} defined while building with cmake (only active with DEBUG_LEVEL=2)
\    #define DEBUG_MSG_${UNIQUE_DEBUG_ID}(msg) std::cerr << \"[${UNIQUE_DEBUG_ID} - \" \\
\                        << __FILENAME__ << ':' \\
\                        << __LINE__ << ':'     \\
\                        << __func__ << \"()]: \"    \\
\                        << msg;
\    //! logging based on the DEBUG_LEVEL_${UNIQUE_DEBUG_ID} defined while building with cmake (active with DEBUG_LEVEL=1 and 2)
\    #define DEBUG_CRIT_MSG_${UNIQUE_DEBUG_ID}(msg) DEBUG_MSG_${UNIQUE_DEBUG_ID}(msg)
\#endif // AUTOGEN_${UNIQUE_DEBUG_ID}_MACROS_H")
else()
    message(FATAL_ERROR "${BoldWhite}${PROJECT_NAME}${ColourReset}: DEBUG_LEVEL_${UNIQUE_DEBUG_ID} is not properly defined as '${DEBUG_LEVEL_${UNIQUE_DEBUG_ID}}', please define it as 0,1 or 2")
endif()

# set the other compiler usually needed compiler flags
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -Wextra -Wfatal-errors -pedantic -g -fPIC")


##########################
## installation properties
##########################

# create the corresponding cmake files to search for the library afterwards
file(WRITE "${CPP_LIB_NAME}-config.cmake"
" get_filename_component(SELF_DIR \"\${CMAKE_CURRENT_LIST_FILE}\" PATH)
\ include(\${SELF_DIR}/\${CMAKE_BUILD_TYPE}/${CPP_LIB_NAME}.cmake)
")

file(WRITE "${CPP_LIB_NAME}-config-version.cmake"
"\n
\ set(PACKAGE_VERSION @${CPP_LIB_NAME}_VERSION@)
\ if(\"\${PACKAGE_FIND_VERSION_MAJOR}\" EQUAL \"@${CPP_LIB_NAME}_VERSION_MAJOR@\")
\     if (\"\${PACKAGE_FIND_VERSION_MINOR}\" EQUAL \"@${CPP_LIB_NAME}_VERSION_MINOR@\")
\         set(PACKAGE_VERSION_EXACT TRUE)
\     elseif(\"\${PACKAGE_FIND_VERSION_MINOR}\" LESS \"@${CPP_LIB_NAME}_VERSION_MINOR@\")
\         set(PACKAGE_VERSION_COMPATIBLE TRUE)
\     else()
\         set(PACKAGE_VERSION_UNSUITABLE TRUE)
\     endif()
\ else()
\     set(PACKAGE_VERSION_UNSUITABLE TRUE)
\ endif()")

# # define library version 
# # Note that the library version is set via FORCE. This prevents users changing the value in the CMakeCache.txt
set(${CPP_LIB_NAME}_VERSION_MAJOR 1 CACHE STRING "major version" FORCE)
set(${CPP_LIB_NAME}_VERSION_MINOR 0 CACHE STRING "minor version" FORCE)
set(${CPP_LIB_NAME}_VERSION ${${CPP_LIB_NAME}_VERSION_MAJOR}.${${CPP_LIB_NAME}_VERSION_MINOR} CACHE STRING "version" FORCE)
# #set(bin_dest "bin")
set(include_dest "include/${CPP_LIB_NAME}-${${CPP_LIB_NAME}_VERSION}")
set(main_lib_dest "lib/${CPP_LIB_NAME}-${${CPP_LIB_NAME}_VERSION}")

install(FILES ${CPP_LIB_NAME}-config.cmake DESTINATION ${main_lib_dest})
# sets the correct version number which find_package() will look for
configure_file(${CPP_LIB_NAME}-config-version.cmake ${CMAKE_CURRENT_BINARY_DIR}/${CPP_LIB_NAME}-config-version.cmake @ONLY)
install(FILES ${CPP_LIB_NAME}-config.cmake ${CMAKE_CURRENT_BINARY_DIR}/${CPP_LIB_NAME}-config-version.cmake DESTINATION ${main_lib_dest})
install(EXPORT ${CPP_LIB_NAME} DESTINATION "${main_lib_dest}")

##########################
## source code options
##########################

find_package(kafi 1.0 REQUIRED)
find_package(connector 1.0 REQUIRED)

# add the code
add_subdirectory( library )

# add tests
if( ENABLE_TESTS_${UNIQUE_DEBUG_ID} )
    message( STATUS "${BoldWhite}${PROJECT_NAME}${ColourReset}: ${BoldGreen}Enabled${ColourReset} the compilation of tests, change with ${BoldWhite}-DENABLE_TESTS_${UNIQUE_DEBUG_ID}=OFF${ColourReset}" )
    add_subdirectory( tests )
else()
    message( STATUS "${BoldWhite}${PROJECT_NAME}${ColourReset}: ${BoldRed}Disabled${ColourReset} the compilation of tests, change with ${BoldWhite}-DENABLE_TESTS_${UNIQUE_DEBUG_ID}=ON${ColourReset}" )
endif()
