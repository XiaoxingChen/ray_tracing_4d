#if !defined(_TEST_RANDOM_FACTORY_H_)
#define _TEST_RANDOM_FACTORY_H_

#include "random_factory.h"
using namespace rtc;

void testRandomFactory()
{
    for(size_t dim = 2; dim < 5; dim++)
    {
        auto vec = random::unitSphere(dim);
        if(vec.size() != dim)
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
}

#endif // _TEST_RANDOM_FACTORY_H_
