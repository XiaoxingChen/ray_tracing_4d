#if !defined(__ECS_MATERIAL_H__)
#define __ECS_MATERIAL_H__

#include "ecs_component.h"
#include "material.h"
#include "mxm/spatial_bvh.h"

namespace rtc
{
namespace ecs
{

class Material: public ComponentBase
{
public:
    virtual ComponentType type() override { return eMaterial; }

    const rtc::GltfMaterial& material() const { return gltf_material_; }
    rtc::GltfMaterial& material() { return gltf_material_; }

protected:
    rtc::GltfMaterial gltf_material_;

};
} // namespace ecs

} // namespace rtc


#endif // __ECS_MATERIAL_H__
