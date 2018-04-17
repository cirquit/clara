#include <arpa/inet.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <random>

#include "../external/object_methods/src/object.h"
#include "../external/Connector/library/server.h"
#include "../external/Connector/library/client.h"

#include "../library/data_association.h"
#include "../library/util.h"

const std::tuple<double, double> parse_object_t(object_t * obj)
{
    const double x_ = std::cos(obj -> angle) * obj -> distance;
    const double y_ = std::sin(obj -> angle) * obj -> distance;

    const double x = x_ * std::cos(obj -> angle_yaw) - y_ * std::sin(obj -> angle_yaw);
    const double y = x_ * std::sin(obj -> angle_yaw) + y_ * std::cos(obj -> angle_yaw);
    return { x + obj -> x_car , y + obj -> y_car};
}

template< size_t N >
void print_data_assoc(clara::data_association<N> & da, int color)
{   
    const std::array<clara::cone_state<double>, N> & cluster = da.get_cluster();

    if (color == 0) { std::cout << "yellow_cone_data = np.array([\n"; }
    if (color == 1) { std::cout << "blue_cone_data = np.array([\n"; }
    if (color == 2) { std::cout << "red_cone_data = np.array([\n"; }
    std::for_each(cluster.begin(), cluster.end(), [&](const clara::cone_state<double> & cs)
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
                      << "[" << 0 << "]"
                      << np_e << ",\n"; 
        }
    });

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

    // maximum amount clusters  for each color, used for preallocation (segfaulting if we go over the limits)
    const size_t N = 400;
    // the data format, our data_association works with
    using raw_cone_data = typename clara::data_association< N >::raw_cone_data;
    // data association for each color
    clara::data_association< N > yellow_data_association;
    clara::data_association< N > blue_data_association;
    clara::data_association< N > red_data_association;
    // saving everything here
    std::vector< std::vector< raw_cone_data > > yellow_cone_data;
    std::vector< std::vector< raw_cone_data > > blue_cone_data;
    std::vector< std::vector< raw_cone_data > > red_cone_data;

    // connection
    const int               carmaker_port = 4401;
    const int               simulink_port = 4402;
    const std::string       simulink_ip   = "10.158.73.210";
    // busy waiting
    connector::server<connector::TCP> from_carmaker_server(carmaker_port);
    from_carmaker_server.init();
    // busy waiting
    connector::client<connector::UDP> to_simulink_client(simulink_port, simulink_ip);
    to_simulink_client.init();


    uint32_t list_size = 0;     // currently seen elements
    size_t   max_elements = 10; // we hardcap the seen elements by 10

    size_t timer = 0;
    object_list_t *objectlist_to_receive = object_list__new(max_elements); // preallocation max_elements

    // send loop
    while ( true ) {
        // busy waiting
        from_carmaker_server.receive_tcp<uint32_t>(list_size);
        // if we get no elements, we are done with the current round
        if (list_size == 0) { break; }
        // if we get more elements than we preallocated, allocate more and save the new maximum
        if (list_size > max_elements) {
            objectlist_to_receive = object_list__new( list_size );
            max_elements = list_size;
        }
        // read all the elements into our preallocated list
        from_carmaker_server.receive_tcp<object_t>(objectlist_to_receive->element[0], sizeof(object_t) * list_size);
        // create new "observation" for this timestamp
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

        const auto & current_used_yellow_clusters = yellow_data_association.get_detected_cluster();
        const auto & current_used_blue_clusters = blue_data_association.get_detected_cluster();
        //std::cout << "got yellow#: " << current_used_yellow_clusters.size() << '\n';
        //std::cout << "got blue#: " << current_used_blue_clusters.size() << '\n';
        // std::mt19937 rng;
        // rng.seed(std::random_device()());
        // std::uniform_int_distribution<std::mt19937::result_type> dist6(1,6);

        if(++timer % 500 != 0){ continue; }

        double * x  = (double *) malloc(sizeof(double) * 2);
        
        for(auto cone : current_used_yellow_clusters)
        {   
            if (cone.is_used()){
                x[0] = cone._mean_vec[0];
                x[1] = cone._mean_vec[1];
                std::cout << "yellow - x: " << x[0] << ", y: " << x[1] << std::endl;
                to_simulink_client.send_udp<double>(x[0], sizeof(double) * 2);
                // usleep(100000);
            }
        }

        for(auto cone : current_used_blue_clusters)
        {
            if (cone.is_used()){
                x[0] = cone._mean_vec[0];
                x[1] = cone._mean_vec[1];
                std::cout << "blue - x: " << x[0] << ", y: " << x[1] << std::endl;
                to_simulink_client.send_udp<double>(x[0], sizeof(double) * 2);
                // usleep(100000);
            }
        }

        free(x);
        // x[0] = static_cast<double>(dist6(rng));
        // x[1] = static_cast<double>(dist6(rng));
    }


}

    //send_thread.join();

    // std::cout << "import numpy as np\n";
    // print_data_assoc< N >(yellow_data_association, 0);
    // print_data_assoc< N >(blue_data_association, 1);

    // print_ground_truth< N >(yellow_cone_data, "yellow");
    // print_ground_truth< N >(blue_cone_data, "blue");
