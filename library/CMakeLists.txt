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

set(SOURCES autogen-${UNIQUE_DEBUG_ID}-macros.h vehicle_state.h clara_clock.h clara-util.h distance_counter.h refined_cones.h lap_counter.h cone_state.h data_association.h csv.h clara.h ../external/object-list/include/object.h)

find_package(LAPACK REQUIRED)
link_directories(${LAPACK_LIBRARIES})
include_directories(${LAPACK_INCLUDE_DIRS})

add_library(${CPP_LIB_NAME} ${SOURCES})
target_link_libraries(${CPP_LIB_NAME} ${LAPACK_LIBRARIES} kafi connector)
set_target_properties(${CPP_LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# used for 'make install'
target_include_directories(${CPP_LIB_NAME} PUBLIC
     $<BUILD_INTERFACE:${CPP_TEMPLATE_SOURCE_DIR}/library> # for headers when building
     $<INSTALL_INTERFACE:${include_dest}>                  # for client in install mode
)

# used for 'make install', too
install(FILES ${SOURCES} DESTINATION "${include_dest}")
install(TARGETS ${CPP_LIB_NAME} EXPORT ${CPP_LIB_NAME} DESTINATION "${main_lib_dest}")

