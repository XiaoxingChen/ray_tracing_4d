#if !defined(_TEST_REGID_BODY_)
#define _TEST_REGID_BODY_

#include "rigid_body.h"

using namespace rtc;

inline void testRigidBody()
{
    for(size_t dim = 2; dim < 5; dim++)
    {
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
        if(dim < 4)
        {
            Ray ray(dim);
            Vec dir(dim);
            dir(0) = 1.;
            // dir(1) = 0.1;
            ray.setDirection(dir);

            std::vector<FloatType> args(4 * dim + 1, 0);
            args.at(0) = 3;
            for(size_t i = dim; i < 2*dim; i++)
                args.at(i) = 1.;
            args.at(2*dim) = 1.;
            args.at(2*dim + 3) = 1.;

            {
                args.at(args.size() - 1) = M_PI * 0.5;
                auto rect = RigidBody::choose(RigidBody::RECTANGLE, dim, args);
                auto record = rect->hit(ray);
                if(!record)
                    throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
                if(record->t != 2)
                    throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
                if(record->p(0) != 2)
                    throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
                if(record->n(0) != -1)
                {
                    std::cout << record->n.str() << std::endl;
                    throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
                }
            }

        }
    }

}

#endif // _TEST_REGID_BODY_
