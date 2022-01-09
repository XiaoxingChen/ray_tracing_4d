#if !defined(__ECS_MESH_H__)
#define __ECS_MESH_H__

#include "ecs_component.h"
#include "mxm/spatial_bvh.h"

namespace rtc
{
namespace ecs
{
template<size_t DIM=3>
class Mesh: public ComponentBase
{
public:
    virtual ComponentType type() override { return eMesh; }

    bvh::PrimitiveMeshTree& meshTree() { return mesh_tree_; }
    const bvh::PrimitiveMeshTree& meshTree() const { return mesh_tree_; }

protected:
    bvh::PrimitiveMeshTree mesh_tree_;

};
} // namespace ecs

} // namespace rtc


#endif // __ECS_MESH_H__
