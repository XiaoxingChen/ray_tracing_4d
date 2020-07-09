#ifndef _CAMERA_H_
#define _CAMERA_H_
#include "vec3.h"
#include "ray.h"

namespace rtc{
class OrientationFixedCamera
{
    public:
        OrientationFixedCamera(V3in position=V3(), V3in focus=V3{500, 500, 0}, V3in c=V3{320, 240, 0})
        :position_(position), f_(focus), c_(c) {}
        Ray pixelRay(int u, int v) const
        {
            auto dir = Vector3{(u - c_.x())/f_.x(),(v - c_.y())/f_.y(), 1.};
            return Ray(position_, dir);
        }
    private:
        Vector3 position_;        
        Vector3 f_;
        Vector3 c_;
};
}//rtc
#endif