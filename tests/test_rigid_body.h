#if !defined(_TEST_REGID_BODY_)
#define _TEST_REGID_BODY_

#include "rigid_body.h"

using namespace rtc;

inline void testRigidBody()
{
    for(size_t dim = 2; dim < 5; dim++)
    {
        Ray ray(dim);
        std::vector<FloatType> args(dim + 1, 0);
        args.at(0) = 3;
        args.at(args.size() - 1) = 1;
        auto sphere4d = RigidBody::choose(RigidBody::SPHERE, dim, args);
        auto record = sphere4d->hit(ray);
        if(!record)
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        if(record->t != 2)
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

}

#endif // _TEST_REGID_BODY_
