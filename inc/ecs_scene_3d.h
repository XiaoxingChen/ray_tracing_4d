#if !defined(_ECS_SCENE_3D_H_)
#define _ECS_SCENE_3D_H_

#include "gltf_utils.h"
#include "ecs_entity.h"
#include "ecs_material.h"
#include "ecs_mesh.h"
#include "ecs_transform.h"


namespace rtc
{
namespace ecs
{

inline EntityPtr sceneGltf3DBox()
{
    tinygltf::Model model;
    const size_t DIM = 3;
    loadModel(model, "assets/box/box.gltf");

    std::shared_ptr<Mat> vertex_buffer = loadMeshAttributes(model, 0, "POSITION");

    auto vertex_index_buffer = std::make_shared<Matrix<size_t>>(std::move(loadMeshIndices(model, 0)));

    auto p_mesh = std::make_shared<rtc::ecs::Mesh>();
    p_mesh->meshTree() = mxm::bvh::PrimitiveMeshTree(vertex_buffer, vertex_index_buffer);
    p_mesh->meshTree().build();

    auto p_texture_buffer = std::make_shared<TextureBuffer>();
    // p_texture_buffer->tex_coord = std::move(*loadMeshAttributes(model, 0, "TEXCOORD_0"));
    p_texture_buffer->tex_coord = Matrix<float>({0,0});
    p_texture_buffer->normal = std::move(*loadMeshAttributes(model, 0, "NORMAL"));
    p_texture_buffer->base_texture = Matrix<Pixel>({1,1}, {Pixel::red()});

    auto p_texture = std::make_shared<rtc::ecs::Material>();
    auto tmp_material = GltfMaterial(p_texture_buffer, vertex_index_buffer, vertex_buffer);
    tmp_material.metalness() = 1.f;
    p_texture->material() = tmp_material;

    auto r = Rotation<float, DIM>::fromAxisAngle(Vec({1,0,0}), M_PI);
    r = Rotation<float, DIM>::fromAxisAngle(Vec({0,1,0}), 3.3 * M_PI_4) * r;

    AffineTransform<float, DIM> tf({0,1.5,10}, r, Vector<float>::ones(DIM));
    auto p_tf = std::make_shared<rtc::ecs::Transform<DIM>>();
    p_tf->transform() = tf;

    EntityPtr root = new Entity("box");
    root->components()[ComponentType::eMaterial] = p_texture;
    root->components()[ComponentType::eMesh] = p_mesh;
    root->components()[ComponentType::eTransform] = p_tf;

    return root;
}

inline EntityPtr sceneGltf3DSphere02()
{
    tinygltf::Model model;
    const size_t DIM = 3;
    loadModel(model, "assets/sphere/sphere.gltf");

    std::shared_ptr<Mat> vertex_buffer = loadMeshAttributes(model, 0, "POSITION");

    auto vertex_index_buffer = std::make_shared<Matrix<size_t>>(std::move(loadMeshIndices(model, 0)));

    auto p_mesh = std::make_shared<rtc::ecs::Mesh>();
    p_mesh->meshTree() = mxm::bvh::PrimitiveMeshTree(vertex_buffer, vertex_index_buffer);
    p_mesh->meshTree().build();

    auto p_texture_buffer = std::make_shared<TextureBuffer>();
    p_texture_buffer->tex_coord = Matrix<float>({0,0});
    p_texture_buffer->normal = std::move(*loadMeshAttributes(model, 0, "NORMAL"));
    p_texture_buffer->base_texture = Matrix<Pixel>({1,1}, {Pixel::white() * 0.2});

    auto p_texture = std::make_shared<rtc::ecs::Material>();
    auto tmp_material = GltfMaterial(p_texture_buffer, vertex_index_buffer, vertex_buffer);
    tmp_material.metalness() = 1.f;
    p_texture->material() = tmp_material;

    auto r = Rotation<float, DIM>::fromAxisAngle(Vec({0,1,0}), M_PI * 0.25);
    // r = Rotation<float, DIM>::fromAxisAngle(Vec({0,1,0}), 3.3 * M_PI_4) * r;



    EntityPtr root = new Entity("world");

    {
        EntityPtr obj = new Entity("obj1");
        obj->components()[ComponentType::eMaterial] = p_texture;
        obj->components()[ComponentType::eMesh] = p_mesh;

        AffineTransform<float, DIM> tf({1,55,10}, r, {50,50,50});
        auto p_tf = std::make_shared<rtc::ecs::Transform<DIM>>();
        p_tf->transform() = tf;
        obj->components()[ComponentType::eTransform] = p_tf;
        root->addChild(obj);
    }

    {
        EntityPtr obj = new Entity("obj2");
        obj->components()[ComponentType::eMaterial] = p_texture;
        obj->components()[ComponentType::eMesh] = p_mesh;

        AffineTransform<float, DIM> tf({2,0,20}, r, {3,3,3});
        auto p_tf = std::make_shared<rtc::ecs::Transform<DIM>>();
        p_tf->transform() = tf;
        obj->components()[ComponentType::eTransform] = p_tf;
        root->addChild(obj);
    }

    return root;
}

inline EntityPtr sceneGltf3DDuck()
{
    tinygltf::Model model;
    const size_t DIM = 3;
    loadModel(model, "build/assets/Duck.gltf");

    std::shared_ptr<Mat> vertex_buffer = loadMeshAttributes(model, 0, "POSITION");

    auto vertex_index_buffer = std::make_shared<Matrix<size_t>>(std::move(loadMeshIndices(model, 0)));
    // (*vertex_buffer) *= 3e-2;
    std::cout << "vertices:" << mxm::to_string((*vertex_buffer)(Block({}, {0, 10})).T()) << std::endl;

    auto p_mesh = std::make_shared<rtc::ecs::Mesh>();
    p_mesh->meshTree() = mxm::bvh::PrimitiveMeshTree(vertex_buffer, vertex_index_buffer);
    p_mesh->meshTree().build();


    // HittableBufferPtr<DIM> buffer = std::make_shared<HittableBuffer<DIM>>();

    auto p_texture_buffer = std::make_shared<TextureBuffer>();
    p_texture_buffer->tex_coord = std::move(*loadMeshAttributes(model, 0, "TEXCOORD_0"));
    p_texture_buffer->normal = std::move(*loadMeshAttributes(model, 0, "NORMAL"));
    p_texture_buffer->base_texture = std::move(*loadMeshTexture(model));

    auto p_texture = std::make_shared<rtc::ecs::Material>();
    auto tmp_material = GltfMaterial(p_texture_buffer, vertex_index_buffer, vertex_buffer);
    tmp_material.metalness() = 0.2;
    p_texture->material() = tmp_material;

    auto r = Rotation<float, DIM>::fromAxisAngle(Vec({1,0,0}), M_PI);
    r = Rotation<float, DIM>::fromAxisAngle(Vec({0,1,0}), 3.3 * M_PI_4) * r;

    AffineTransform<float, DIM> tf({0,1.5,10}, r, Vector<float>::ones(3) * 3e-2);
    auto p_tf = std::make_shared<rtc::ecs::Transform<DIM>>();
    p_tf->transform() = tf;

    EntityPtr root = new Entity("duck");
    root->components()[ComponentType::eMaterial] = p_texture;
    root->components()[ComponentType::eMesh] = p_mesh;
    root->components()[ComponentType::eTransform] = p_tf;

    // buffer->push_back(Hittable<DIM>(
    //     RigidBody<DIM>::createPrimitiveMesh(
    //         Vec({0,1.5,10}), r, vertex_buffer, vertex_index_buffer),
    //     p_texture));


    // AcceleratedHitManager<DIM> manager;
    // auto root = std::shared_ptr<bvh::Node<DIM>>(new bvh::Node<DIM>(DIM, buffer, {0, buffer->size()}));
    // root->split(1, /*verbose*/ false);
    // manager.setRoot(root);

    return root;
}

} // namespace ecs

} // namespace rtc



#endif // _ECS_SCENE_3D_H_
