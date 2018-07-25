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

#ifndef CLARA_OBJ
#define CLARA_OBJ

#include "../external/object-list/include/object.h"

namespace clara {
    namespace object {
        struct cone_position {
            double x_pos;
            double y_pos;
        };

        struct clara_obj {
            double x_pos;
            double y_pos;
            double yaw;

            struct cone_position basecase_yellow_cones[2];
            struct cone_position basecase_blue_cones[2];
            bool go_grittr_flag;
            bool valid_cones;
            object_list_t object_list;
            object_list_t clustered_object_list;
        };
    }
}

#endif // CLARA_OBJ
