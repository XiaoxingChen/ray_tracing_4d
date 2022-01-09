#if !defined(__ECS_TRANSFORM_H__)
#define __ECS_TRANSFORM_H__

#include "ecs_component.h"
#include "mxm/transform_affine.h"

namespace rtc
{
namespace ecs
{
template<size_t DIM=3>
class Transform: public ComponentBase
{
public:
    virtual ComponentType type() override { return eTransform; }
    mxm::AffineTransform<float, DIM> & transform() { return tf_; }
    const mxm::AffineTransform<float, DIM> & transform() const { return tf_; }

protected:
    mxm::AffineTransform<float, DIM> tf_;

};
} // namespace ecs

} // namespace rtc


#endif // __ECS_TRANSFORM_H__
