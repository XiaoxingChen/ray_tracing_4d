#if !defined(_UTILS_H_)
#define _UTILS_H_

#include "io.h"
#include <vector>
#include <iostream>
#include "camera.h"
#include "random_factory.h"
#include "ThreadPool.h"
#include "material.h"
#include "rigid_body.h"
#include "hittable.h"
#include "ray_tracer.h"
#include "scenes.h"

namespace rtc
{
std::vector<Pixel> threadFunc(
    const Camera& cam,
    const HitManager& manager,
    int sample_num,
    PixelCoordinates::const_iterator begin,
    PixelCoordinates::const_iterator end)
{
    std::vector<Pixel> ret;
    for(auto it = begin; it != end; it++)
    {
        auto & uv = *it;
        Pixel col;
        auto r = cam.pixelRay(uv);
        for (int s = 0; s < sample_num; s++)
        {
            col += trace(manager, r, 0);
        }
        col *= (1./float(sample_num));
        for(int i = 0; i < 3; i++) col(i) = sqrt(col(i));
        ret.push_back(col);
    }
    return ret;
}

} // namespace rtc


#endif // _UTILS_H_
