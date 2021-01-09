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
        min_(min_pt), max_(max_pt) { checkDimension(__FILE__, __LINE__); }
    AxisAlignedBoundingBox(const std::vector<FloatType>& min_pt, const std::vector<FloatType>& max_pt):
        min_(min_pt), max_(max_pt) { checkDimension(__FILE__, __LINE__); }

    AxisAlignedBoundingBox(size_t dim):
        min_(dim), max_(dim) { clear(); }

    const Vec& min() const {return min_;}
    const Vec& max() const {return max_;}
    bool empty() const { return min_(0) > max_(0); }
    bool clear()
    {
        min_ = Vec::ones(min_.size()) * INFINITY;
        max_ = Vec::ones(max_.size()) * (-INFINITY);
    }

    void extend(const Mat& vertices)
    {
        if(vertices.shape(0) != min_.size())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        for(size_t i = 0; i < vertices.shape(0); i++)
        {
            for(size_t j = 0; j < vertices.shape(1); j++)
            {
                if(vertices(i, j) > max_(i)) max_(i) = vertices(i, j);
                if(vertices(i, j) < min_(i)) min_(i) = vertices(i, j);
            }
        }
    }

    void checkDimension(const char* file, uint32_t line) const
    {
        if(min_.size() != max_.size())
            throw std::runtime_error(std::string(file) + ":" + std::to_string(line));
    }

    bool hit(const Ray& ray) const
    {
        if(empty()) return false;
        auto min_max = AxisAlignedBoundingBox::hit(ray, min_, max_);
        return min_max[0] < min_max[1];
    }

    static std::array<FloatType, 2> hit(const Ray& ray, const Vec& vertex_min, const Vec& vertex_max)
    {
        if(ray.origin().size() != vertex_min.size())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        if(ray.origin().size() != vertex_max.size())
            throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

        FloatType t_min = -INFINITY;
        FloatType t_max = INFINITY;
        for(int i = 0; i < vertex_min.size(); i++)
        {
            FloatType t0 = (vertex_min(i) - ray.origin()(i)) / ray.direction()(i);
            FloatType t1 = (vertex_max(i) - ray.origin()(i)) / ray.direction()(i);
            t_min = std::max(t_min, std::min(t0, t1));
            t_max = std::min(t_max, std::max(t0, t1));
        }
        return {t_min, t_max};
    }

  private:
    Vec min_;
    Vec max_;
};
} // namespace rtc



#endif // _AXIS_ALIGNED_BOUNDING_BOX_

