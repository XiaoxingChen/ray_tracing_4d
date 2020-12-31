#if !defined(_TEST_LINALG_H)
#define _TEST_LINALG_H

#include <iostream>
#include "linalg.h"

using namespace rtc;

void testLinearAlgebra()
{
    Mat m1({3,3},{1,1,1, 2,2,2, 3,3,3});
    Vec v1({1,1,1});
    UnitVec u1(v1);
    Mat expected({3,1}, {3, 6, 9});
    std::cout << v1.str() << std::endl;

    const UnitVec& u2(u1);

    if(fabs(u2(2) - sqrt(1./3)) > eps())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    if(v1.str() != "1.000000 \n1.000000 \n1.000000 \n")
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    if((m1.matmul(v1) - expected).norm() > eps())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    if((u1 - v1 * sqrt(1./3)).norm() > eps())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
}

#endif // _TEST_LINALG_H
