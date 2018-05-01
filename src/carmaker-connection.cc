#include <arpa/inet.h>
#include <iostream>
#include <mutex>
#include <random>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>

#include "../external/Connector/library/client.h"
#include "../external/Connector/library/server.h"
#include "../external/object-list/include/object.h"

#include "../library/data_association.h"
#include "../library/util.h"

const std::tuple< double, double > get_car_pos( object_t *obj )
{   // add yaw
    return { obj->x_car , obj->y_car };
} 

const std::tuple< double, double > parse_object_t( object_t *obj ) {
    const double x_ = std::cos( obj->angle ) * obj->distance;
    const double y_ = std::sin( obj->angle ) * obj->distance;

    const double x = x_ * std::cos( obj->angle_yaw ) - y_ * std::sin( obj->angle_yaw );
    const double y = x_ * std::sin( obj->angle_yaw ) + y_ * std::cos( obj->angle_yaw );
    return {x + obj->x_car, y + obj->y_car};
}

template < typename T >
void print_data_assoc( clara::data_association< T > &da, int color ) {
    const std::vector< clara::cone_state< double > > &cluster = da.get_cluster( );

    if ( color == 0 ) {
        std::cout << "yellow_cone_data = np.array([\n";
    }
    if ( color == 1 ) {
        std::cout << "blue_cone_data = np.array([\n";
    }
    if ( color == 2 ) {
        std::cout << "red_cone_data = np.array([\n";
    }
    clara::util::for_each_( cluster, [&]( const clara::cone_state< T > &cs ) {
        const T mean_x = cs._mean_vec[ 0 ];
        const T mean_y = cs._mean_vec[ 1 ];
        const T cov_xx = cs._cov_mat[ 0 ];
        const T cov_yy = cs._cov_mat[ 3 ];
        const T cov_xy = cs._cov_mat[ 1 ];

        std::string np_b = "np.array([";
        std::string np_e = "])";
        if ( mean_x != 0 || mean_y != 0 ) {

            std::cout << np_b << "[" << mean_x << ", " << mean_y << "]"
                      << ", "
                      << "[[" << cov_xx << ", " << cov_xy << "]"
                      << ", "
                      << "[" << cov_xy << ", " << cov_yy << "]]"
                      << ", "
                      << "[" << 0 << "]" << np_e << ",\n";
        }
    } );

    std::cout << "]);\n";
}

template< typename T >
void print_ground_truth(std::vector< std::vector< typename clara::data_association< T
>::raw_cone_data> > & cones, std::string id)
{
    std::cout << '\n' << id << "_cone_ground_truth = np.array([";
    for(auto cur_timestamp : cones)
    {
        for(auto cone : cur_timestamp)
        {
            std::cout << '[' << std::get<0>(cone) << ", " << std::get<1>(cone) << "],\n";
        }
    }
    std::cout << "]);\n";
}

int main( ) {

    // the data format, our data_association works with
    using raw_cone_data = typename clara::data_association< double >::raw_cone_data;
    // parametrization of data associtaion
    const size_t preallocated_cluster_count           = 500;
    const size_t preallocated_detected_cones_per_step = 10;
    const double max_distance_btw_cones_m             = 2;  // meter
    const double variance_xx                          = 0.25;
    const double variance_yy                          = 0.25;
    const size_t apply_variance_step_count            = 4;
    const int    cluster_search_range                 = 5; // +/- to the min/max used cluster-index

    // data association for each color
    clara::data_association< double > yellow_data_association(
        preallocated_cluster_count, preallocated_detected_cones_per_step, max_distance_btw_cones_m,
        variance_xx, variance_yy, apply_variance_step_count, cluster_search_range );
    clara::data_association< double > blue_data_association(
        preallocated_cluster_count, preallocated_detected_cones_per_step, max_distance_btw_cones_m,
        variance_xx, variance_yy, apply_variance_step_count, cluster_search_range );
    clara::data_association< double > red_data_association(
        preallocated_cluster_count, preallocated_detected_cones_per_step, max_distance_btw_cones_m,
        variance_xx, variance_yy, apply_variance_step_count, cluster_search_range );

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
        // busy waiting
        from_carmaker_server.receive_tcp< uint32_t >( list_size );

        // if we get no elements, we are done with the current round
        // if (list_size == 0) { break; }

        // read all the elements into our preallocated list
        from_carmaker_server.receive_tcp< object_t >( objectlist_to_receive.element[ 0 ],
                                                      sizeof( object_t ) * list_size );
        // create new "observation" for this timestamp
        // yellow_cone_data.push_back( std::vector< raw_cone_data >( ) );
        // blue_cone_data.push_back( std::vector< raw_cone_data >( ) );
        // red_cone_data.push_back( std::vector< raw_cone_data >( ) );

        // if ( ++timer % 500 != 0 ) {
        //     continue;
        // }
        

        // for all seen cones
        for ( uint32_t i = 0; i < list_size; ++i ) {

            object_t *          el   = &objectlist_to_receive.element[ i ];
            const raw_cone_data cone = parse_object_t( el );

            if ( el->type == 0 ) {
                raw_yellow_cones.emplace_back( cone );
            } else if ( el->type == 1 ) {
                raw_blue_cones.emplace_back( cone );
            } else {
                raw_red_cones.emplace_back( cone );
            }
        }

        if (objectlist_to_receive.element[0].type == 4) { break; }

        // const auto & all_yellow_cones = yellow_data_association.classify_new_data( yellow_cone_data.back( ) );
        // const auto & all_blue_cones = blue_data_association.classify_new_data( blue_cone_data.back( ) );

        const auto & all_yellow_cones = yellow_data_association.classify_new_data( raw_yellow_cones );
        const auto & all_blue_cones = blue_data_association.classify_new_data( raw_blue_cones );
        UNUSED(all_yellow_cones);
        UNUSED(all_blue_cones);

        // const std::tuple<double, double> car_pos = get_car_pos(&objectlist_to_receive.element[ 0 ]);

        // const clara::cone_state<double> & min_yellow_cone = *std::min_element(all_yellow_cones.begin(), all_yellow_cones.end(),
        //      [&](const clara::cone_state<double> & cone_a, const clara::cone_state<double> & cone_b)
        //      {
        //         return cone_a.distance(car_pos) < cone_b.distance(car_pos);
        //      });

        // const clara::cone_state<double> & min_blue_cone = *std::min_element(all_blue_cones.begin(), all_blue_cones.end(),
        //      [&](const clara::cone_state<double> & cone_a, const clara::cone_state<double> & cone_b)
        //      {
        //         return cone_a.distance(car_pos) < cone_b.distance(car_pos);
        //      });

        // const std::tuple<double, double> yellow_cone_pos = min_yellow_cone.get_approx_position();
        // const clara::cone_state<double> & neighbor_blue_cone = *std::min_element(all_blue_cones.begin(), all_blue_cones.end(),
        //      [&](const clara::cone_state<double> & cone_a, const clara::cone_state<double> & cone_b)
        //      {
        //         return cone_a.distance(yellow_cone_pos) < cone_b.distance(yellow_cone_pos);
        //      });

//        const auto current_used_yellow_clusters = yellow_data_association.get_detected_cluster( );
//        const auto current_used_blue_clusters   = blue_data_association.get_detected_cluster( );

        // const auto & yellow_cluster = yellow_data_association.get_cluster();
        // const auto & blue_cluster = blue_data_association.get_cluster();
        
        // const auto & cur_yellow_cluster_ix = yellow_data_association.get_detected_cluster_ixs();
        // const auto & cur_blue_cluster_ix = blue_data_association.get_detected_cluster_ixs();
        



        // if ( ++timer % 750 != 0 ) {
        //     continue;
        // }

        // double *x = (double *)malloc( sizeof( double ) * 2 );

        // const double dist_yellow = min_yellow_cone.distance(car_pos);
        // const double dist_blue = min_blue_cone.distance(car_pos);

        // x[ 0 ] = dist_yellow;
        // x[ 1 ] = dist_blue;

        // to_simulink_client.send_udp< double >( x[ 0 ], sizeof( double ) * 2 );


        // for(auto y_ix : cur_yellow_cluster_ix)
        // {
        //     auto cone = yellow_cluster[y_ix];
        //     x[ 0 ] = cone._mean_vec[ 0 ];
        //     x[ 1 ] = cone._mean_vec[ 1 ];
        //     // std::cout << "yellow - x: " << x[0] << ", y: " << x[1] << std::endl;
        //     to_simulink_client.send_udp< double >( x[ 0 ], sizeof( double ) * 2 );
        // }

        // for(auto b_ix : cur_blue_cluster_ix)
        // {
        //     auto cone = blue_cluster[b_ix];
        //     x[ 0 ] = cone._mean_vec[ 0 ];
        //     x[ 1 ] = cone._mean_vec[ 1 ];
        //     // std::cout << "yellow - x: " << x[0] << ", y: " << x[1] << std::endl;
        //     to_simulink_client.send_udp< double >( x[ 0 ], sizeof( double ) * 2 );
        // }

        // for ( auto cone : current_used_yellow_clusters ) {
        //     if ( cone.is_used( ) ) {
        //         x[ 0 ] = cone._mean_vec[ 0 ];
        //         x[ 1 ] = cone._mean_vec[ 1 ];
        //         // std::cout << "yellow - x: " << x[0] << ", y: " << x[1] << std::endl;
        //         to_simulink_client.send_udp< double >( x[ 0 ], sizeof( double ) * 2 );
        //         // usleep(100000);
        //     }
        // }

        // for ( auto cone : current_used_blue_clusters ) {
        //     if ( cone.is_used( ) ) {
        //         x[ 0 ] = cone._mean_vec[ 0 ];
        //         x[ 1 ] = cone._mean_vec[ 1 ];
        //         // std::cout << "blue - x: " << x[0] << ", y: " << x[1] << std::endl;
        //         to_simulink_client.send_udp< double >( x[ 0 ], sizeof( double ) * 2 );
        //         // usleep(100000);
        //     }
        // }

        // free( x );
       
    }
        // print out the python file
    std::cout << "import numpy as np\n";
    print_data_assoc< double >(yellow_data_association, 0);
    print_data_assoc< double >(blue_data_association, 1);
    print_data_assoc< double >(red_data_association, 2);
}
