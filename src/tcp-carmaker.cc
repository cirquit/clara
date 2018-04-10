#include <arpa/inet.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>

#include "../external/object_methods/src/object.h"
#include "../library/data_association.h"
#include "../library/util.h"

const std::tuple<double, double> parse_object_t(object_t * obj)
{
    const double x_ = std::cos(obj -> angle) * obj -> distance;
    const double y_ = std::sin(obj -> angle) * obj -> distance;

    const double x = x_ * std::cos(obj -> angle_yaw) - y_ * std::sin(obj -> angle_yaw);
    const double y = x_ * std::sin(obj -> angle_yaw) + y_ * std::cos(obj -> angle_yaw);
    return { x, y };
}

template< size_t N >
void print_data_assoc(clara::data_association<N> & da, int color)
{   
    const std::array<clara::cone_state<double>, N> & cluster = da.get_current_cluster();
    const std::array<double, N> & weights = da.get_cone_state_weights();

    if (color == 0) { std::cout << "yellow_cone_data = np.array([\n"; }
    if (color == 1) { std::cout << "blue_cone_data = np.array([\n"; }
    if (color == 2) { std::cout << "red_cone_data = np.array([\n"; }
    clara::util::zipWith_([&](const clara::cone_state<double> & cs, double cluster_weight)
    {
        double mean_x = cs._mean_vec[0];
        double mean_y = cs._mean_vec[1];
        double cov_xx = cs._cov_mat[0];
        double cov_yy = cs._cov_mat[3];
        if (cs._observations.size() < 4)
        {
            cov_xx -= 0.25;
            cov_yy -= 0.25;
        }
        double cov_xy = cs._cov_mat[1];

        std::string np_b = "np.array([";
        std::string np_e = "])";
        if ( mean_x != 0 || mean_y != 0 ) {

            std::cout << np_b
                      << "[" << mean_x << ", " << mean_y << "]"  << ", "
                      << "[[" << cov_xx << ", " << cov_xy << "]" << ", "
                      << "[" << cov_xy << ", " << cov_yy << "]]" << ", "
                      << "["  << cluster_weight << "]"
                      << np_e << ",\n"; 
        }
    }, cluster.begin(), cluster.end(), weights.begin());

    std::cout << "]);\n";
}

template< size_t N >
void print_ground_truth(std::vector< std::vector< typename clara::data_association< N >::raw_cone_data> > & cones, std::string id)
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

    int                listen_socket, client_socket, bind_ret;
    struct sockaddr_in server_addr, client_addr;

    // creating socket
    listen_socket = socket( AF_INET, SOCK_STREAM, 0 );
    if ( listen_socket == -1 ) {
        std::cerr << "-- Error creating socket with -1\n";
        return 1;
    } else {
        std::cerr << "-- Created socket!\n";
    }

    // binding port
    server_addr.sin_family      = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port        = htons( 4401 );
    bind_ret = bind( listen_socket, (struct sockaddr *)&server_addr, sizeof( server_addr ) );
    if ( bind_ret < 0 ) {
        std::cerr << "-- Error binding with error: " << bind_ret << '\n';
        return 1;
    } else {
        std::cerr << "-- Binding successful!\n";
    }

    // connecting to socket, allowing a client to connect
    listen( listen_socket, 1 );
    int len       = sizeof( struct sockaddr_in );
    client_socket = accept( listen_socket, (struct sockaddr *)&client_addr, (socklen_t *)&len );
    if ( client_socket < 0 ) {
        std::cerr << "-- Connection accept failed with error: " << client_socket << '\n';
        return 1;
    } else {
        std::cerr << "-- Client connected!\n";
    }

    // maximum 400 clusters currently for each color, this is up to test the
    // performance of vector on unbounded data
    const size_t N = 400;
    // the data format, our data_association works with
    using raw_cone_data = typename clara::data_association< N >::raw_cone_data;


    clara::data_association< N > yellow_data_association;
    clara::data_association< N > blue_data_association;

    std::vector< std::vector< raw_cone_data > > yellow_cone_data;
    std::vector< std::vector< raw_cone_data > > blue_cone_data;
    std::vector< std::vector< raw_cone_data > > red_cone_data;

    // create read-do-something loop
    uint32_t       list_size = 0;
    object_list_t *objectlist_to_receive;
    int i = 0;
    while ( ++i < 40000 ) {

        int read_size = recv( client_socket, &list_size, sizeof( uint32_t ), 0 );
        objectlist_to_receive = object_list__new( list_size );

        size_t objectlist_size = sizeof( object_t ) * list_size;
        read_size = recv( client_socket, objectlist_to_receive->element, objectlist_size, 0 );

        yellow_cone_data.push_back( std::vector< raw_cone_data >( ) );
        blue_cone_data.push_back( std::vector< raw_cone_data >( ) );
        red_cone_data.push_back( std::vector< raw_cone_data >( ) );

        // for all seen cones
        for ( uint32_t i = 0; i < list_size; ++i ) {
            
            object_t * el = &objectlist_to_receive->element[ i ];
            const raw_cone_data cone = parse_object_t(el);

            if(el -> type == 0) {
                yellow_cone_data.back().push_back(cone);
            } else if (el -> type == 1) {
                blue_cone_data.back().push_back(cone);
            } else {
                red_cone_data.back().push_back(cone);
            }
        }

        yellow_data_association.classify_new_data( yellow_cone_data.back() );
        blue_data_association.classify_new_data( blue_cone_data.back() );

//        std::cout << std::endl;
    }

    std::cout << "import numpy as np\n";
    print_data_assoc< N >(yellow_data_association, 0);
    print_data_assoc< N >(blue_data_association, 1);

    print_ground_truth< N >(yellow_cone_data, "yellow");
    print_ground_truth< N >(blue_cone_data, "blue");


}