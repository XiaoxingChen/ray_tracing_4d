#if !defined(_TEST_RAY_H_)
#define _TEST_RAY_H_

#include "ray.h"
using namespace rtc;

inline void testRay()
{
    Ray ray1({0,0,0}, {1,1,1});
    // const Ray& ray2(ray1);

    // if(ray1.direction()(1) != ray2.direction()(1))
    //     throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    if(!ray1.valid(500))
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
}

#endif // _TEST_RAY_H_
