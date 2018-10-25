#include <arpa/inet.h>
#include <iostream>
#include <mutex>
#include <random>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <connector-1.0/client.h>
#include <connector-1.0/server.h>

#include "../external/object-list/include/object.h"
#include "../library/clara.h"

// naive rotation formula, is already included in clara.h but can be called here to get the ground truth cones with the real car position `x_car` and `y_car`
const std::tuple< double, double > parse_object_t( object_t *obj ) {
    const double x_ = std::cos( obj->angle ) * obj->distance;
    const double y_ = std::sin( obj->angle ) * obj->distance;

    const double x = x_ * std::cos( obj->angle_yaw ) - y_ * std::sin( obj->angle_yaw );
    const double y = x_ * std::sin( obj->angle_yaw ) + y_ * std::cos( obj->angle_yaw );
    return {x + obj->x_car, y + obj->y_car};
}

int main( ) {

    // the data format, our data_association works with
    using raw_cone_data = typename clara::data_association< double >::raw_cone_data;

    // saving everything here
    std::vector< std::vector< raw_cone_data > > yellow_cone_data;
    std::vector< std::vector< raw_cone_data > > blue_cone_data;
    std::vector< std::vector< raw_cone_data > > red_cone_data;

    // connection
    const int         carmaker_port = 4401;
    const int         simulink_port = 4402;
    const std::string simulink_ip   = "10.158.78.191";
    // busy waiting
    connector::server< connector::TCP > from_carmaker_server( carmaker_port );
    from_carmaker_server.init( );
    // busy waiting
    connector::client< connector::UDP > to_simulink_client( simulink_port, simulink_ip );
    to_simulink_client.init( );

    // preallocation of the receiving list
    uint32_t list_size    = 0;   // currently seen elements
    object_list_t objectlist_to_receive;

    std::vector< raw_cone_data > raw_yellow_cones;
    std::vector< raw_cone_data > raw_blue_cones;
    std::vector< raw_cone_data > raw_red_cones;
    raw_yellow_cones.reserve(10);
    raw_blue_cones.reserve(10);
    raw_red_cones.reserve(10);
    
    // send loop
    while ( true ) {

        raw_yellow_cones.clear();
        raw_blue_cones.clear();
        raw_red_cones.clear();

        from_carmaker_server.receive_tcp< uint32_t >( list_size );

        // read all the elements into our preallocated list
        from_carmaker_server.receive_tcp< object_t >( objectlist_to_receive.element[ 0 ],
                                                      sizeof( object_t ) * list_size );

        // if we get a cone with this "invalid" type, we stop the simulation
        if (objectlist_to_receive.element[0].type == 4) { break; }
    }
}
