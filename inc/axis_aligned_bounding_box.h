#if !defined(_AXIS_ALIGNED_BOUNDING_BOX_)
#define _AXIS_ALIGNED_BOUNDING_BOX_

#include "ray.h"
#include <algorithm>


namespace rtc
{
class AxisAlignedBoundingBox
{
  public:
    AxisAlignedBoundingBox(Vec min_pt, Vec max_pt):
    min_(min_pt), max_(max_pt){}
    AxisAlignedBoundingBox(const std::vector<FloatType>& min_pt, const std::vector<FloatType>& max_pt):
    min_(min_pt), max_(max_pt){}

    const Vec& min() const {return min_;}
    const Vec& max() const {return max_;}

    bool hit(const Ray& ray) const
    {
        float_t t_min = -INFINITY;
        float_t t_max = INFINITY;
        for(int i = 0; i < 3; i++)
        {
            t_min = std::max(t_min, (min_(i) - ray.origin()(i)) / ray.direction()(i));
            t_max = std::min(t_max, (max_(i) - ray.origin()(i)) / ray.direction()(i));
        }
        return t_min < t_max;
    }

  private:
    Vec min_;
    Vec max_;
};
} // namespace rtc



#endif // _AXIS_ALIGNED_BOUNDING_BOX_
