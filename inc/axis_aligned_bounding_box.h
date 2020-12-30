#if !defined(_AXIS_ALIGNED_BOUNDING_BOX_)
#define _AXIS_ALIGNED_BOUNDING_BOX_

#include "ray.h"
#include <algorithm>


namespace rtc
{
class AxisAlignedBoundingBox
{
  public:
    AxisAlignedBoundingBox(Vector3 min_pt, Vector3 max_pt):
    min_(min_pt), max_(max_pt){}

    const Vector3& min() const {return min_;}
    const Vector3& max() const {return max_;}

    bool hit(const Ray& ray) const
    {
        float_t t_min = -INFINITY;
        float_t t_max = INFINITY;
        for(int i = 0; i < 3; i++)
        {
            t_min = std::max(t_min, (min_.at(i) - ray.origin().at(i)) / ray.direction().at(i));
            t_max = std::min(t_max, (max_.at(i) - ray.origin().at(i)) / ray.direction().at(i));
        }
        return t_min < t_max;
    }

  private:
    Vector3 min_;
    Vector3 max_;
};
} // namespace rtc



#endif // _AXIS_ALIGNED_BOUNDING_BOX_
