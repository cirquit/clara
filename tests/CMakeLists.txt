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

# set(SOURCES catch.h csv.h main.cc cone_state_tests.cc util_tests.cc)

# add_executable(${TEST_NAME} ${SOURCES})
# target_link_libraries(${TEST_NAME} ${LIB_NAME})

# custom data association test for debugging purposes

set(TEST_NAME_CLARA_CARMAKER clara_carmaker_simulation)
set(SOURCES_CLARA_CARMAKER clara_carmaker_simulation.cc)
add_executable(${TEST_NAME_CLARA_CARMAKER} ${SOURCES_CLARA_CARMAKER})
target_link_libraries(${TEST_NAME_CLARA_CARMAKER} ${LIB_NAME} connector)

set(TEST_NAME_CLARA_REPLAY clara_csv_replay)
set(SOURCES_CLARA csv.h clara_csv_replay.cc)
add_executable(${TEST_NAME_CLARA_REPLAY} ${SOURCES_CLARA})
target_link_libraries(${TEST_NAME_CLARA_REPLAY} ${LIB_NAME})

set(TEST_NAME_UTIL clara_util_test)
set(SOURCES_UTIL catch.h main.cc csv.h util_tests.cc)
add_executable(${TEST_NAME_UTIL} ${SOURCES_UTIL})
target_link_libraries(${TEST_NAME_UTIL} ${LIB_NAME})

# custom data association test for debugging purposes
set(TEST_NAME_LOOPING clara_looping_test)
set(SOURCES_LOOPING catch.h main.cc looping_tests.cc)
add_executable(${TEST_NAME_LOOPING} ${SOURCES_LOOPING})
target_link_libraries(${TEST_NAME_LOOPING} ${LIB_NAME})

# custom data association test for debugging purposes
set(TEST_NAME_MEMORY clara_memory)
set(SOURCES_MEMORY catch.h main.cc memory_tests.cc)
add_executable(${TEST_NAME_MEMORY} ${SOURCES_MEMORY})
target_link_libraries(${TEST_NAME_MEMORY} ${LIB_NAME})

# file(COPY example-data DESTINATION .)  # execute ./clara_test
# file(COPY example-data DESTINATION ..) # execute ./tests/clara_test
# file(COPY visualization DESTINATION ..) #
