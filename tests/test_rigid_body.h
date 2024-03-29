#if !defined(_TEST_REGID_BODY_)
#define _TEST_REGID_BODY_

#include "rigid_body.h"
#include "mxm/geometry_primitive.h"
#include "mxm/string.h"
#include <iostream>



using namespace rtc;

inline void testRectangle2D(int k = 1)
{
    size_t dim = 2;
    Ray<> ray(Vec({0, 3}), Vec({1,-1}));

    FloatType angle = M_PI * (0.25 + 0.5 * k);
    std::vector<FloatType> args{3,0, 1,1, 1,0, 0,1, angle};

    auto rect = RigidBody<2>::choose(RigidBody<2>::RECTANGLE, dim, args);
    auto record = rect->hit(ray);
    if(!record)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    FloatType expect_t = 3 * sqrt(2) - 1;
    if(fabs(record->t - expect_t) > eps() * 10)
    {
        std::cout << "expect: " << expect_t << ", get: " << record->t << std::endl;
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    Vec expect_p({3.f - sqrt(.5f), sqrt(.5f)});
    if((record->p - expect_p).norm() > eps() * 10)
    {
        std::cout << "expect: " << mxm::to_string(expect_p) << ", get: " << mxm::to_string(record->p) << std::endl;
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    if((record->n - Vec({-sqrt(.5f), sqrt(.5f)})).norm() > 10 * eps())
    {
        std::cout << mxm::to_string(record->n) << std::endl;
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
}

inline void testRectangle3D()
{
    size_t dim = 3;
    std::vector<uint32_t> dir_raw{0xbe018a73,0xbdae120b,0x3f7d0269};
    std::vector<FloatType> dir_f;
    for(size_t i = 0; i < dim ; i++)
    {
        float val = *(float*)&dir_raw[i];
        dir_f.push_back(val);
    }
    Ray<> ray(Vec({0, 0, 0}), Vec(dir_f));

    std::vector<FloatType> args{0.,0, 15, 2,2,2, 1,0, 0,1, 0,0, 0};

    auto rect = RigidBody<3>::choose(RigidBody<3>::RECTANGLE, dim, args);
    auto record = rect->hit(ray);
    if(!record)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
}

template<size_t DIM>
void testSphericalRigidBody()
{
    Ray<> ray(DIM);
    std::vector<FloatType> args(DIM + 1, 0);
    args.at(0) = 3;
    args.at(args.size() - 1) = 1;
    auto sphere4d = RigidBody<DIM>::choose(RigidBody<DIM>::SPHERE, DIM, args);
    auto record = sphere4d->hit(ray);
    if(!record)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    if(record->t != 2)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
}

inline void testRigidBody()
{
    testSphericalRigidBody<2>();
    testSphericalRigidBody<3>();
    testSphericalRigidBody<4>();

    for(int k = 0; k < 10; k++)
        testRectangle2D(k);

    testRectangle3D();

}

inline void testIntersectEquation2D()
{
    Mat line_seg({2,2}, {0, 1, 0, 1});
    Ray<> ray({1,0}, {-1, 1});

    Vec result(intersectEquation(line_seg, ray));
    Vec expect({sqrt(0.5f), .5f});
    if((result - expect).norm() > eps())
    {
        std::cout << mxm::to_string(result.T()) << std::endl;
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
}

inline void testIntersectEquationXD(size_t dim)
{
    Mat line_seg(Mat::identity(dim));
    Ray<> ray(Vec::zeros(dim), Vec::ones(dim));

    Vec result(intersectEquation(line_seg, ray));
    Vec expect(Vec::ones(dim) * (1./dim));
    expect(0) = sqrt(expect(0));
    if((result - expect).norm() > eps())
    {
        std::cout << mxm::to_string(result.T()) << std::endl;
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
}

inline void testPutTriangleInPlane()
{
    Mat triangle({3,3}, {0.804559827, 0.804559827, 0.785871685, -0.0360073522, -0.186628193, -0.0360073522, 5.50387716, 5.50387716, 5.48705053});
    Vec hit_p({0.803415656, -0.17609112, 5.50284719});

    Mat triangle_2d;
    Vec hit_p_2d;
    putTriangleInPlane(triangle, hit_p, triangle_2d, hit_p_2d);
}


inline void testPrimitiveGeometry()
{
    testIntersectEquation2D();
    for(size_t dim = 2; dim < 5; dim++)
        testIntersectEquationXD(dim);
    testPutTriangleInPlane();
}

#endif // _TEST_REGID_BODY_
