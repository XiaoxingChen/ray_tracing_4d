#if !defined(_TEST_PIXEL_H_)
#define _TEST_PIXEL_H_

#include <iostream>
#include "linalg.h"
#include "mxm/cv_pixel.h"
#include "accessor.h"

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

    {
        auto img = loadImage("assets/sky_box/star_night.jpeg");
        if(img.shape() != Shape({750,499}))
        {
            std::cout << img.shape(0) << "," << img.shape(1) << std::endl;
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        }
    }
    {
        //4,17,33
        auto img = imread<uint8_t, 3>("assets/sky_box/star_night.jpeg");
        auto px = img(0,0);
        if(px != PixelType<uint8_t, 3>({4,17,33}))
        {
            std::cout << px.str() << std::endl;
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        }

        // px = img(263,475);
        px = img(475, 263);
        if(px != PixelType<uint8_t, 3>({50,66,89}))
        {
            std::cout << (uint32_t)px(0) << "," << (uint32_t)px(1) << "," << (uint32_t)px(2) << std::endl;
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
        }
    }

    {
        auto img = imread<uint8_t, 3>("assets/sky_box/star_night.jpeg");
        imwrite("build/imwrite_test_01.png", img);
    }

    {
        auto img = imread<float, 3>("assets/sky_box/star_night.jpeg");
        imwrite("build/imwrite_test_02.png", img);
    }
}


#endif // _TEST_PIXEL_H_
