#if !defined(__ECS_ENTITY_TREE__)
#define __ECS_ENTITY_TREE__

#include "ecs_entity.h"
#include "mxm/spatial_bvh.h"
#include "ecs_transform.h"
#include "ecs_mesh.h"
#include "ecs_material.h"
#include "mxm/geometry_ray.h"

namespace rtc
{

namespace ecs
{

struct HitRecord: public mxm::bvh::PrimitiveMeshTree::HitRecord
{
    using BaseType = mxm::bvh::PrimitiveMeshTree::HitRecord;
    HitRecord(const BaseType& base_obj): BaseType(base_obj){}
    HitRecord(){}
    EntityPtr entity;
    float t_global;
};

template<size_t DIM>
mxm::AffineTransform<float, DIM>
getTransform(const EntityPtr node)
{
    if(node->components().count(ComponentType::eTransform))
    {
        auto pTfBase = node->components().at(ComponentType::eTransform).get();
        auto pTf = static_cast<Transform<DIM>*>(pTfBase);
        return pTf->transform();
    }
    return AffineTransform<float, DIM>::identity();
}

template<size_t DIM>
mxm::AffineTransform<float, DIM>
transformUpFromDown(const EntityPtr ancestor, const EntityPtr descendant)
{
    auto ret = AffineTransform<float, DIM>::identity();
    if(ancestor == descendant) return ret;
    EntityPtr tmp_ptr = descendant;
    for(size_t i = 0; i < 99; i++)
    {
        ret = ret * getTransform<DIM>(tmp_ptr);
        if(tmp_ptr->parent() == ancestor)
        {
            return ret;
        }
    }
    assert(false);
    return AffineTransform<float, DIM>::identity();
}

template<size_t DIM>
std::vector<HitRecord>
hit(const EntityPtr root, const mxm::Ray& ray_in, mxm::bvh::HitType hit_type)
{
    std::vector<HitRecord> records;
    mxm::Ray ray(ray_in);

    if(nullptr == root) return records;

    std::stack<mxm::AffineTransform<float, DIM>> tf_stk;
    std::stack<EntityPtr> entity_stk;

    // prepare stack

    entity_stk.push(root);
    tf_stk.push(getTransform<DIM>(root));

    // traverse
    while(!entity_stk.empty())
    {
        auto target_entity = entity_stk.top();
        entity_stk.pop();

        auto target_tf = tf_stk.top();
        tf_stk.pop();

        auto target_ray = apply(target_tf.inv(), ray);
        auto t_scale = mxm::norm(target_tf.linear().matmul(target_ray.direction())); // local to world

        // ray cast
        if(target_entity->components().count(ComponentType::eMesh))
        {
            rtc::ecs::Mesh<DIM>* pMesh = static_cast<rtc::ecs::Mesh<DIM>*>(target_entity->components().at(ComponentType::eMesh).get());
            auto base_records = pMesh->meshTree().hit(target_ray, hit_type);
            for(const auto & base_record: base_records)
            {
                records.push_back(HitRecord(base_record));
                records.back().t_global = base_record.t * t_scale;
                records.back().entity = target_entity;
                if(hit_type == mxm::bvh::HitType::eClosestHit)
                {
                    ray.tMax() = std::min(records.back().t_global, ray.tMax());
                }
            }
            if(hit_type == mxm::bvh::HitType::eAnyHit && records.size() > 0) break;
        }

        // process children
        for(auto name_child_pair : target_entity->children())
        {
            entity_stk.push(name_child_pair.second);
            tf_stk.push(getTransform<DIM>(name_child_pair.second) * target_tf);
        }
    }

    // finish traversal, deal with hit type
    if(hit_type == mxm::bvh::HitType::eMultiHit) return records;
    if(hit_type == mxm::bvh::HitType::eAnyHit) return records;
    if(hit_type == mxm::bvh::HitType::eClosestHit)
    {
        std::sort(records.begin(), records.end(),
            [](const HitRecord& lhs, const HitRecord& rhs)
            {
                return lhs.t_global < rhs.t_global;
            });
        if (!records.empty())
        {
            // std::cout << "#" ;
            records.resize(1);
        }
    }
    return records;

}

template<size_t DIM=3>
void materialPostprocess(const HitRecord& record, Pixel& color, Ray& scatter_ray)
{
    // local frame calculation
    Vector<float> local_hit_p = record.ray(record.t);
    Matrix<float> prim = static_cast<Mesh<DIM>*>(record.entity->components().at(ComponentType::eMesh).get())->meshTree().primitive(record.prim_idx);
    Vector<float> local_hit_n = mxm::primitiveNorm(prim, record.ray);

    AffineTransform<float, DIM> tf_world_from_local = transformUpFromDown<DIM>(record.entity->root(), record.entity);

    if(record.entity->components().count(ComponentType::eMaterial) == 0)
    {
        color = Pixel::black();
        scatter_ray = apply(tf_world_from_local, record.ray);
        return;
    }
    auto pComponentBase = record.entity->components().at(ComponentType::eMaterial).get();
    Material* pMaterial = static_cast<Material*>(pComponentBase);


    Ray local_ray = pMaterial->material().scatter(record.ray, record.prim_idx, local_hit_p, local_hit_n);


    scatter_ray = apply(tf_world_from_local, local_ray); // result 1

    color = pMaterial->material().attenuation(record.prim_idx, local_hit_p); //result 2
    // color = Pixel::red();
}

void clear(EntityPtr& root)
{
    if(root->isLeaf())
    {
        delete root;
        root = nullptr;
        return;
    }
    for(auto & name_child_pair : root->children())
    {
        clear(name_child_pair.second);
    }
    root->children().clear();
}


} // namespace ecs


} // namespace rtc



#endif // __ECS_ENTITY_TREE__
