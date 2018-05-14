
#include <vector>
#include <iostream>

#include "catch.h"
#include "../library/clara-util.h"

std::vector<int> loop(std::vector<int> & vec, int min_ix, int max_ix, int bounds)
{
    std::vector<int> result;

    int distance = max_ix - min_ix + 2 * bounds;

    int max_elem = 0;
    int ix = 0;
    int min_diff = min_ix - bounds;
    if (min_diff < 0)
        ix = vec.size() + min_diff;
    else
        ix = min_diff;

    for(int i = 0; i < distance; i++)
    {
        int my_ix = (ix + i) % vec.size();
        result.push_back(vec[my_ix]);
    }
    return result;
}

void print_vec(std::vector<int> & vec)
{
    std::cout << "[ ";
    std::for_each(vec.begin(), vec.end(), [](int x){ std::cout << x << " "; });
    std::cout << "] \n";
}


TEST_CASE( "looping_vector_test", "[loop]" ) {

    // create a 100 element vector with each index equal to its element
    std::vector<int> vector_100(100, 0);
    int counter = 0;
    std::transform(vector_100.begin(), vector_100.end(), vector_100.begin(), [&](int e){ UNUSED(e); return counter++; });

    // create a similar 10 element vector
    std::vector<int> vector_10(10, 0);
    counter = 0;
    std::transform(vector_10.begin(), vector_10.end(), vector_10.begin(), [&](int e){ UNUSED(e); return counter++; });

    // we look 8 elements infront / back
    int bounds = 8;

    // test for underflow of min_ix = -3 
    // should return [ 97 98 99 0 1 2 3 4 | 5 6 7 8 9 | 10 11 12 13 14 15 16 17 ]
    //                     -bounds             mid           +bounds
    auto vec_01 = loop(vector_100, 5, 10, bounds);
    std::cout << "vec_01 = ";
    print_vec(vec_01);
    REQUIRE(vec_01.size() == 21);

    // test for under/overflow and out of bounds by max_ix = 18 and min_ix = -5
    // should return [ 7 8 9 0 1 2 3 4 | 5 6 7 8 9 | 0 1 2 3 4 5 6 7 ]
    //                   -bounds            mid         +bounds   
    auto vec_02 = loop(vector_10, 5, 10, bounds);
    std::cout << "vec_02 = ";
    print_vec(vec_02);
    REQUIRE(vec_02.size() == 21);

    // test for overflow of max_ix = 102
    // should return [ 72 73 74 75 76 77 78 79 | 80 81 82 83 84 85 86 87 88 89 90 91 92 93 | 94 95 96 97 98 99 0 1 ] 
    //                   -bounds                                mid                              +bounds   
    auto vec_03 = loop(vector_100, 80, 94, bounds);
    std::cout << "vec_03 = ";
    print_vec(vec_03);
    REQUIRE(vec_03.size() == 30);



}
