#if !defined(_TEST_IMAGE_PROCESSING_H_)
#define _TEST_IMAGE_PROCESSING_H_

#include "image_processing.h"

using namespace rtc;

inline void testImageProcessing()
{
    {
        auto img = imread<float, 3>("assets/sky_box/star_night.jpeg");
        decltype(img) img_block = img(Block({0,128},{0,128}));
        // decltype(img) img_block = img;
        // std::cout << "img_block(2,3): " << img_block(2,3).str() << std::endl;
        Shape new_shape({32,32});
        auto scaled_img = resize(img_block, new_shape);
        imwrite("build/image_scale_test_01_before.png", img_block);
        imwrite("build/image_scale_test_01_after.png", scaled_img);
    }
}


#endif // _TEST_IMAGE_PROCESSING_H_
