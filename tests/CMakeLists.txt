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

set(SOURCES catch.h csv.h main.cc cone_state_tests.cc util_tests.cc data_association_tests.cc)

add_executable(${TEST_NAME} ${SOURCES})
target_link_libraries(${TEST_NAME} ${LIB_NAME})

file(COPY test-data DESTINATION .)  # execute ./kafi_tests
file(COPY test-data DESTINATION ..) # execute ./tests/kafi_tests
