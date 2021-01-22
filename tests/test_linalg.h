#if !defined(_TEST_LINALG_H)
#define _TEST_LINALG_H

#include <iostream>
#include "linalg.h"

using namespace rtc;

inline void testPixel()
{
    Pixel color({1,1,1});
    if(color.rU8() != 0xff)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    Pixel px2(color);
    if(px2.rU8() != 0xff)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    std::vector<Pixel> img(10, px2);
    if(img.back().rU8() != 0xff)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    std::vector<Pixel> img2(img);
    img.insert(img.end(), img2.begin(), img2.begin() + 5);
    if(img.back().rU8() != 0xff)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

}

inline void testOrthogonalComplement()
{
    {
        Mat in({2,3},{0,0,1, 0,1,0});

        if(in(Row(0)).matmul(orthogonalComplement(in).T())(0,0) > eps())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        if(in(Row(1)).matmul(orthogonalComplement(in.T()))(0,0) > eps())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    if(0){
        Mat in({1,3},{1,0,0});
        Mat complement = orthogonalComplement(in);
        Vec out1 = complement(Row(0));
        Vec out2 = complement(Row(1));

        if(static_cast<Vec>(in).dot(out1) > eps())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        if(static_cast<Vec>(in).dot(out2) > eps())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

}

inline void testSolveLowerTriangle()
{
    Mat L({3,3}, {1,0,0, 2,3,0, 4,5,6});
    Vec b({2,3,4});
    Vec expect({2, -1./3, -7./18});
    Vec x = solveLowerTriangle(L, b);
    if((x - expect).norm() > eps())
    {
        std::cout << x.T().str() << std::endl;
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    Vec expect_u({-4./9, -1./9, 2./3});
    Vec x_u = solveUpperTriangle(L.T(), b);
    if((x_u - expect_u).norm() > eps())
    {
        std::cout << x_u.T().str() << std::endl;
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

}

inline void testQRcalcMatQ()
{
    return;
    Mat mat_a({3,3},{0, 0, 0, 0.431263924, -1, -1, 0.902225852, 0, -1});
    Mat expect_q({3,3},
    {0.00000000e+00,  5.55111512e-17, -1.00000000e+00,
    -4.31263911e-01, -9.02225825e-01, -5.55111512e-17,
    -9.02225825e-01,  4.31263911e-01,  1.11022302e-16});
    Mat mat_q = qr::calcMatQ(mat_a);
    if((mat_q - expect_q).norm() > 2 * eps())
    {
        std::cout << mat_q.str() << std::endl;
        std::cout << "norm: " << (mat_q - expect_q).norm() << std::endl;
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
}

inline void testQRSolve()
{
    // using namespace qr;
    Mat mat_a({3,3}, {12, -51, 4, 6, 167, -68, -4, 24, -41});
    Mat expect_q({3,3}, {6./7, -69./175, -58./175, 3./7, 158./175, 6./175, -2./7, 6./35, -33./35});
    Mat mat_q = qr::calcMatQ(mat_a);
    if((mat_q - expect_q).norm() > 2 * eps())
    {
        std::cout << mat_q.str() << std::endl;
        std::cout << "norm: " << (mat_q - expect_q).norm() << std::endl;
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    Vec b({2,3,9});
    Vec expect({-0.10244898, -0.08359184, -0.25844898});
    Vec x(qr::solve(mat_a, b));

    if((x - expect).norm() > eps())
    {
        std::cout << x.T().str() << std::endl;
        // std::cout << "norm: " << (mat_q - expect_q).norm() << std::endl;
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
}

inline Mat testMatRefTransposeReturn(size_t dim)
{
    return Mat::Identity(dim).T();
}

inline void testMatRef()
{
    {
        auto mat = testMatRefTransposeReturn(3);
        if(mat(0,0) != 1.f)
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    // {
    //     Mat mat_a(Mat::ones({3,3}));
    //     mat_a(Col(2)) = Vec::zeros(3);
    //     std::cout << mat_a(Col(2)).T().str() << std::endl;
    // }

    // {
    //     const Mat mat_a(Mat::ones({3,3}));
    //     mat_a(Col(2)) = Vec::zeros(3);
    //     std::cout << mat_a(Col(2)).T().str() << std::endl;
    // }

}

inline void testComplexBase()
{
    Complex a({1,2});
    Complex b({3,1});
    Complex expected({1,7});

    std::cout << a.str() << std::endl;
    if((a * b - expected).norm() > eps())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
}

inline void testLinearAlgebra()
{
    Mat m1({3,3},{1,1,1, 2,2,2, 3,3,3});
    Vec v1({1,1,1});
    UnitVec u1(v1);
    Mat expected({3,1}, {3, 6, 9});
    std::cout << v1.str() << std::endl;

    const UnitVec& u2(u1);

    if(fabs(u2(2) - sqrt(1./3)) > eps())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    if(v1.str() != "1.000000\n1.000000\n1.000000\n")
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    if((m1.matmul(v1) - expected).norm() > eps())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    if((u1 - v1 * sqrt(1./3)).norm() > eps())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    if(fabs(Mat::Identity(3).det() - 1.) > eps())
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    testPixel();
    testMatRef();
    testOrthogonalComplement();
    testSolveLowerTriangle();
    testQRcalcMatQ();
    testQRSolve();
    testComplexBase();

}


#endif // _TEST_LINALG_H
