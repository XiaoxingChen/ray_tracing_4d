#include "axis_aligned_bounding_box.h"
#include <string>


using namespace rtc;

void testAABB()
{
    AxisAlignedBoundingBox box({1,1,1}, {2,2,2});
    Ray ray1({0,0,0}, {1,1,1});
    Ray ray2({0,0,0}, {1,1,2.001});

    if(!box.hit(ray1))
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    if(box.hit(ray2))
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
}
