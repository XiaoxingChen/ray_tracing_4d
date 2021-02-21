#if !defined(_SCENES_H_)
#define _SCENES_H_

#include "material.h"
#include "rigid_body.h"
#include "hittable.h"
#include "bounding_volume_hierarchy.h"
#include "gltf_utils.h"
#include "primitive_mesh_tree.h"
#include "rigid_transform.h"

namespace rtc
{
namespace scene
{

inline HitManager simple2D_001()
{
    HitManager manager;
    size_t dim = 2;

    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({6.5, 10}), Rotation::Identity(dim), {1}),
        Material::choose(Material::DIELECTRIC, Pixel({0.5, 0.5, 0.5})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({1.5,13}), Rotation::Identity(dim), {1}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.6, 0.6, 0.4})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({4.5,15}), Rotation::Identity(dim), {1}),
        Material::choose(Material::METAL));
    manager.addHittables(
    RigidBody::choose(RigidBody::SPHERE, Vec({-100, 30}), Rotation::Identity(dim), {100}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8})));

    return manager;
}
#if 0
inline HitManager simple3D_001()
{
    HitManager manager;

    size_t dim = 3;

    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({0, 1, 5}), Rotation::Identity(dim), {1}),
        Material::choose(Material::DIELECTRIC, Pixel({0.5, 0.5, 0.5})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({3,-1,7}), Rotation::Identity(dim), {1}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.6, 0.6, 0.4})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({-3,-1,7}), Rotation::Identity(dim), {1}),
        Material::choose(Material::METAL));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({0.,-100,30}), Rotation::Identity(dim), {100}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8})));

    manager.addHittables(
        RigidBody::choose(RigidBody::RECTANGLE, Vec({0.,2, 15}), Rotation::fromPlaneAngle(Vec({1,0,0}), Vec({0,1,1}), 0.9), {2,2,2}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.3, 0.3})));

    return manager;
}

inline HitManager simple3D_002()
{
    std::cout << "simple3D_002 was deprecated" << std::endl;
    HitManager manager;

    size_t dim = 3;

    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({0.,-100,30}), Rotation::Identity(dim), {100}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8})));

    Mat vertices({3,8}, {0,0,0, 1,0,0, 1,0,1, 0,0,1, 0,1,0, 1,1,0, 1,1,1, 0,1,1}, 1);
    std::vector<std::vector<size_t>> indices({
        {0,1,2}, {0,2,3},
        {1,5,6}, {1,6,2},
        {3,2,6}, {3,6,7},
        {4,0,3}, {4,3,7},
        {0,4,5}, {0,5,1},
        {4,7,6}, {4,6,5}});

    manager.addHittables(
        RigidBody::createPrimitiveMesh(Vec({0, 0, 5}),
            Rotation::fromAxisAngle(Vec({1,0,1}), 0.5),
            // Rotation::Identity(dim),
            vertices, indices),
        // Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.2}))
            Material::choose(Material::METAL)
        );

    return manager;
}

inline AcceleratedHitManager simple3D_003()
{
    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();

    size_t dim = 3;

    buffer->push_back(Hittable(
        RigidBody::choose(RigidBody::SPHERE, Vec({0, -1, 5}), Rotation::Identity(dim), {1}),
        Material::choose(Material::DIELECTRIC, Pixel({0.5, 0.5, 0.5})), "glass"));
    buffer->push_back(Hittable(
        RigidBody::choose(RigidBody::SPHERE, Vec({3,1,7}), Rotation::Identity(dim), {1}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.6, 0.6, 0.4})), "ball"));
    buffer->push_back(Hittable(
        RigidBody::choose(RigidBody::SPHERE, Vec({-3,1,7}), Rotation::Identity(dim), {1}),
        Material::choose(Material::METAL), "iron ball"));
    buffer->push_back(Hittable(
        RigidBody::choose(RigidBody::SPHERE, Vec({0.,100,30}), Rotation::Identity(dim), {100}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8})), "ground"));

    buffer->push_back(Hittable(
        RigidBody::choose(RigidBody::RECTANGLE, Vec({0.,-2, 15}), Rotation::fromPlaneAngle(Vec({1,0,0}), Vec({0,1,1}), 0.9), {2,2,2}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.3, 0.3})), "cube"));

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;
}

inline AcceleratedHitManager simple3D_004()
{
    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();

    size_t dim = 3;

    Rotation r(Rotation::fromPlaneAngle(Vec({1,0,0}), Vec({0,1,1}), 0.));
    Vec trans({2, 0, 5});
    std::shared_ptr<Mat> vertex_buffer(new Mat({3,8}, {0,0,0, 1,0,0, 1,0,1, 0,0,1, 0,1,0, 1,1,0, 1,1,1, 0,1,1}, Mat::COL));
    *vertex_buffer = r.apply(*vertex_buffer);
    for(size_t i = 0; i < vertex_buffer->shape(1); i++)
        (*vertex_buffer)(Col(i)) += trans;
    std::vector<std::vector<size_t>> indices({
        {0,1,2}, {0,2,3},
        {1,5,6}, {1,6,2},
        {3,2,6}, {3,6,7},
        {4,0,3}, {4,3,7},
        {0,4,5}, {0,5,1},
        {4,7,6}, {4,6,5}});

    for(auto & idx: indices)
    {
        buffer->push_back(Hittable(
            RigidBody::createPolygonPrimitive(vertex_buffer, idx),
            // Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8}))));
            Material::choose(Material::METAL)));
    }

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;
}

inline AcceleratedHitManager gltf3DTriangle()
{
    tinygltf::Model model;
    size_t dim = 3;
    loadModel(model, "assets/triangle/triangle.gltf");

    std::shared_ptr<Mat> vertex_buffer = loadMeshVertices(model, 0);

    std::vector<std::vector<size_t>> indices;
    for(size_t i = 0; i < vertex_buffer->shape(1) / 3; i++)
        indices.push_back({3 * i, 3 * i + 1, 3 * i + 2});

    (*vertex_buffer)(Row(2)) += 3;

    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();
    for(auto & idx: indices)
    {
        buffer->push_back(Hittable(
            RigidBody::createPolygonPrimitive(vertex_buffer, idx),
            Material::choose(Material::METAL)));
    }

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;

}
#endif
inline AcceleratedHitManager gltf3DBox()
{
    tinygltf::Model model;
    size_t dim = 3;
    loadModel(model, "assets/box/box.gltf");

    std::shared_ptr<Mat> vertex_buffer = loadMeshAttributes(model, 0, "POSITION");

    auto vertex_index_buffer = std::make_shared<Matrix<size_t>>(std::move(loadMeshIndices(model, 0)));


    std::cout << "Vertices: \n" << vertex_buffer->T().str();


    Rotation r(Rotation::fromPlaneAngle(Vec({1,0,0}), Vec({0,1,1}), 0.9));
    *vertex_buffer = r.apply(*vertex_buffer);
    (*vertex_buffer)(Row(2)) += 3;

    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();
    for(size_t prim_idx = 0; prim_idx < vertex_index_buffer->shape(1); prim_idx++)
    {
        buffer->push_back(Hittable(
            RigidBody::createPolygonPrimitive(vertex_buffer, vertex_index_buffer, prim_idx),
            Material::choose(Material::LAMBERTIAN, Pixel({0.3, 0.3, 0.6}))));
    }

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;

}
#if 1
inline AcceleratedHitManager gltf3DSphere()
{
    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();
    size_t dim = 3;

    {
        tinygltf::Model model;

        loadModel(model, "assets/sphere/sphere.gltf");

        std::shared_ptr<Mat> vertex_buffer = loadMeshVertices(model, 0);
        auto vertex_index_buffer = std::make_shared<Matrix<size_t>>(std::move(loadMeshIndices(model, 0)));
        auto p_texture_buffer = std::make_shared<TextureBuffer>();
        p_texture_buffer->tex_coord = std::move(*loadMeshAttributes(model, 0, "TEXCOORD_0"));
        p_texture_buffer->base_texture = Matrix<Pixel>({1,1});
        p_texture_buffer->base_texture(0,0) = Pixel({0.3, 0.3, 0.6});
        p_texture_buffer->normal = std::move(*loadMeshAttributes(model, 0, "NORMAL"));

        auto p_texture = std::shared_ptr<Material>(
            new GltfTexture(p_texture_buffer, vertex_index_buffer, vertex_buffer));

        buffer->push_back(Hittable(
            RigidBody::createPrimitiveMesh(Vec({-1.2,0,4}), Rotation::Identity(dim) , vertex_buffer, vertex_index_buffer),
            p_texture));

        buffer->push_back(Hittable(
            RigidBody::createPrimitiveMesh(Vec({1.2,0,4}), Rotation::Identity(dim) , vertex_buffer, vertex_index_buffer),
            p_texture));
    }

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1, /*verbose*/ false);
    manager.setRoot(root);

    return manager;

}
#endif
inline AcceleratedHitManager gltf3DBoomBox()
{
    tinygltf::Model model;
    size_t dim = 3;
    loadModel(model, "build/assets/BoomBox.gltf");

    std::shared_ptr<Mat> vertex_buffer = loadMeshVertices(model, 0);

    auto vertex_index_buffer = std::make_shared<Matrix<size_t>>(std::move(loadMeshIndices(model, 0)));
    (*vertex_buffer) *= 100.;
    std::cout << "vertices:" << (*vertex_buffer)(Block({}, {0, 10})).T().str() << std::endl;

    Rotation r(Rotation::fromPlaneAngle(Vec({0,1,0}), Vec({1,0,0}), M_PI));
    r = Rotation::fromPlaneAngle(Vec({1,0,0}), Vec({0,0,1}), -M_PI/6) * r;
    *vertex_buffer = r.apply(*vertex_buffer);

    (*vertex_buffer)(Row(2)) += 6;

    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();


    auto p_texture_buffer = std::make_shared<TextureBuffer>();
    p_texture_buffer->tex_coord = std::move(*loadMeshAttributes(model, 0, "TEXCOORD_0"));
    p_texture_buffer->base_texture = std::move(*loadMeshTexture(model));

    std::cout << "vertex_index_buffer shape: " << vertex_index_buffer->shape(1) << std::endl;

    auto p_texture = std::shared_ptr<Material>(
        new GltfTexture(p_texture_buffer, vertex_index_buffer, vertex_buffer));

    for(size_t prim_idx = 0; prim_idx < vertex_index_buffer->shape(1); prim_idx++)
    {
        buffer->push_back(Hittable(
            RigidBody::createPolygonPrimitive(vertex_buffer, vertex_index_buffer, prim_idx),
            p_texture
            ));
    }

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1, /*verbose*/ false);
    manager.setRoot(root);

    return manager;

}

inline AcceleratedHitManager simple3DPrism()
{
    size_t dim = 3;
    // auto vertex_buffer = std::shared_ptr<Mat>(new Mat({2,4},{1,1, 1,0, 0,0, 0,1}, Mat::COL));
    // auto vertex_index_buffer = std::shared_ptr<Matrix<size_t>>(new Matrix<size_t>({2,4},{0,1, 1,2, 2,3, 3,0}, Mat::COL));

    // auto vertex_buffer = std::shared_ptr<Mat>(new Mat({2,3},{.5,.5, .5,-.5, -.5,-.5}, Mat::COL));
    // auto vertex_index_buffer = std::shared_ptr<Matrix<size_t>>(new Matrix<size_t>({2,3},{0,1, 1,2, 2,0}, Mat::COL));

    auto vertex_buffer = std::shared_ptr<Mat>(new Mat({2,8},{0,0, 0,1, 1,1, 1,0, .8,0, .8,.5, .2,.5, .2,0}, Mat::COL));
    auto vertex_index_buffer = std::shared_ptr<Matrix<size_t>>(new Matrix<size_t>({2,8},{0,1, 1,2, 2,3, 3,4, 4,5, 5,6, 6,7, 7,0}, Mat::COL));

    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();
    buffer->push_back(Hittable(
            RigidBody::createPrism(Vec({-.5, 0, 3}), Rotation::fromAxisAngle(Vec({1,1,1}), -.5*M_PI_2), 1.0, vertex_buffer, vertex_index_buffer),
            Material::choose(Material::METAL, Pixel({0.5, 0.3, 0.3}))
            ));

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1, /*verbose*/ false);
    manager.setRoot(root);

    return manager;
}

inline AcceleratedHitManager gltf3DDuck()
{
    tinygltf::Model model;
    size_t dim = 3;
    loadModel(model, "build/assets/Duck.gltf");

    std::shared_ptr<Mat> vertex_buffer = loadMeshAttributes(model, 0, "POSITION");

    auto vertex_index_buffer = std::make_shared<Matrix<size_t>>(std::move(loadMeshIndices(model, 0)));
    (*vertex_buffer) *= 5e-3;
    std::cout << "vertices:" << (*vertex_buffer)(Block({}, {0, 10})).T().str() << std::endl;

    (*vertex_buffer)(Row(2)) += 3;

    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();

    auto p_texture_buffer = std::make_shared<TextureBuffer>();
    p_texture_buffer->tex_coord = std::move(*loadMeshAttributes(model, 0, "TEXCOORD_0"));
    p_texture_buffer->base_texture = std::move(*loadMeshTexture(model));

    auto p_texture = std::shared_ptr<Material>(
        new GltfTexture(p_texture_buffer, vertex_index_buffer, vertex_buffer));

    for(size_t prim_idx = 0; prim_idx < vertex_index_buffer->shape(1); prim_idx++)
    {
        buffer->push_back(Hittable(
            RigidBody::createPolygonPrimitive(vertex_buffer, vertex_index_buffer, prim_idx),
            p_texture
            ));
    }

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1, /*verbose*/ false);
    manager.setRoot(root);

    return manager;

}
#if 0
inline AcceleratedHitManager rectangle3D_001()
{
    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();

    size_t dim = 3;

    buffer->push_back(Hittable(
        RigidBody::choose(RigidBody::RECTANGLE, Vec({0.,3, 15}), Rotation::fromPlaneAngle(Vec({1,0,0}), Vec({0,1,0}), M_PI_2), {2,2,2}),
        // Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.3, 0.3}))));
        Material::choose(Material::METAL), "cube"));

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;
}

inline AcceleratedHitManager rectangle3D_002()
{
    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();

    size_t dim = 3;

    buffer->push_back(Hittable(
        RigidBody::choose(RigidBody::RECTANGLE, Vec({0.,2, 15}), Rotation::fromPlaneAngle(Vec({1,0,0}), Vec({0,1,1}), 0.9), {2,2,2}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.3, 0.3})), "cube"));

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;
}
#endif
inline HitManager simple4D_001()
{
    HitManager manager;
    size_t dim = 4;

    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({5, 0, 0, 18}), Rotation::Identity(dim), {3}),
        Material::choose(Material::DIELECTRIC, Pixel({0.5, 0.5, 0.5})));
    manager.addHittables(
        RigidBody::choose(RigidBody::SPHERE, Vec({0.,105,0, 30}), Rotation::Identity(dim), {100}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.5, 0.5, 0.8})));

    Rotation rec_ori(Rotation::fromPlaneAngle(Vec({1,0,0,0}), Vec({0,0,0,1}), M_PI / 3));
    Vec rec_pos({-3, 0, 0, 15});

    manager.addHittables(
        RigidBody::choose(RigidBody::RECTANGLE, rec_pos, rec_ori, {2,-2,2,0.0001}),
        Material::choose(Material::METAL));

    manager.addHittables(
        RigidBody::choose(RigidBody::RECTANGLE, rec_pos, rec_ori, {1,-1,1,0.0002}),
        Material::choose(Material::LAMBERTIAN, Pixel({0.2, 0.6, 0.2})));

    return manager;
}

inline AcceleratedHitManager gltf4DBox()
{
    tinygltf::Model model;
    size_t dim = 4;
    loadModel(model, "assets/box/box.gltf");

    std::shared_ptr<Mat> vertex_buffer_3d = loadMeshAttributes(model, 0, "POSITION");
    std::shared_ptr<Mat> vertex_buffer = std::make_shared<Mat>(Shape({4, vertex_buffer_3d->shape(1) + 1}));
    vertex_buffer->setBlock(0,0, *vertex_buffer_3d);
    (*vertex_buffer)(vertex_buffer->shape(0) - 1, vertex_buffer->shape(1) - 1) = .1;

    size_t last_vertex_idx = vertex_buffer->shape(1) - 1;

    auto indices_3d = loadMeshIndices(model, 0);
    auto vertex_index_buffer = std::make_shared<Matrix<size_t>>(Shape{dim, indices_3d.shape(1)});
    vertex_index_buffer->setBlock(0,0, indices_3d);
    (*vertex_index_buffer)(Row(end()-1)) *= 0;
    (*vertex_index_buffer)(Row(end()-1)) += last_vertex_idx;

    // std::cout << "Vertices: \n" << vertex_buffer->str();

    Rotation r(Rotation::fromPlaneAngle(Vec({0,0,0,1}), Vec({0,0,0,1}), M_PI_4));

    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();

    buffer->push_back(Hittable(
        RigidBody::createPrimitiveMesh(Vec({0,0,0,5}),r, vertex_buffer, vertex_index_buffer),
        Material::choose(Material::LAMBERTIAN, Pixel({0.3, 0.3, 0.6}))));

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;

}

inline Hittable loadTexturelessPrism4D(
    const RigidTrans& pose, const std::string& gltf_path, FloatType height=1.)
{
    tinygltf::Model model;
    size_t dim = 4;
    loadModel(model, gltf_path);

    std::shared_ptr<Mat> vertex_buffer_3d = loadMeshAttributes(model, 0, "POSITION");
    auto vertex_index_buffer_3d = std::make_shared<Matrix<size_t>>(loadMeshIndices(model, 0));

    return Hittable(
        RigidBody::createPrism(pose.translation(), pose.rotation(), height, vertex_buffer_3d, vertex_index_buffer_3d),
        Material::choose(Material::METAL, Pixel({0.8, 0.8, 0.8})));
}

inline Hittable loadPrism4D(
    const RigidTrans& pose, const std::string& gltf_path, FloatType scale=1.0, FloatType height=1.)
{
    tinygltf::Model model;
    size_t dim = 4;
    loadModel(model, gltf_path);

    std::shared_ptr<Mat> vertex_buffer_3d = loadMeshAttributes(model, 0, "POSITION");
    (*vertex_buffer_3d) *= scale;
    auto vertex_index_buffer_3d = std::make_shared<Matrix<size_t>>(loadMeshIndices(model, 0));

    auto p_texture_buffer = std::make_shared<TextureBuffer>();
    p_texture_buffer->tex_coord = std::move(*loadMeshAttributes(model, 0, "TEXCOORD_0"));
    p_texture_buffer->base_texture = std::move(*loadMeshTexture(model));
    p_texture_buffer->normal = std::move(*loadMeshAttributes(model, 0, "NORMAL"));

    auto p_texture = std::shared_ptr<Material>(
        new GltfTexture(p_texture_buffer, vertex_index_buffer_3d, vertex_buffer_3d));

    return Hittable(
        RigidBody::createPrism(pose.translation(), pose.rotation(), height, vertex_buffer_3d, vertex_index_buffer_3d),
        p_texture);
}

inline AcceleratedHitManager gltf4DBoxPrism()
{
    size_t dim = 4;

    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();
    #if 0
    buffer->push_back(loadTexturelessPrism4D(
        RigidTrans(
            Vec({0,0,0,6}), Rotation::fromPlaneAngle(Vec({0,0,0,1}), Vec({0,0,0,1}), 1*M_PI_4)),
            "assets/sphere/sphere.gltf", 3));

    buffer->push_back(loadPrism4D(
        RigidTrans(
            Vec({0,-1,0,10}), Rotation::fromPlaneAngle(Vec({0,0,0,1}), Vec({0,0,0,1}), 1*M_PI_4)),
            "build/assets/Duck.gltf", 3e-2, 3));
    #endif
    buffer->push_back(loadPrism4D(
        RigidTrans(
            Vec({0,-1,0,10}), Rotation::fromPlaneAngle(Vec({1,0,0,0}), Vec({0,0,0,1}), 1.5*M_PI_4)),
            "build/assets/BoomBox.gltf", 200., 0.1));

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;
}

inline void To4DMesh(
    std::shared_ptr<Mat>& vertex_3d,
    const std::vector<std::vector<size_t>>& index_3d,
    std::shared_ptr<Mat>& vertex_4d,
    std::vector<std::vector<size_t>>& index_4d,
    FloatType thickness=1e-4)
{
    size_t offset = vertex_3d->shape(1);
    vertex_4d = std::shared_ptr<Mat>(new Mat({4, vertex_3d->shape(1) * 2}));
    vertex_4d->setBlock(0, 0, *vertex_3d);
    vertex_4d->setBlock(0, offset, *vertex_3d);
    (*vertex_4d)(Col(3)) *= 0;
    (*vertex_4d)(Block({3,}, {offset, })) += thickness;

    index_4d.clear();
    for(auto & tri_index: index_3d)
    {
        index_4d.push_back({tri_index.at(0), tri_index.at(1), tri_index.at(2), tri_index.at(0) + offset});
        index_4d.push_back({tri_index.at(1), tri_index.at(2), tri_index.at(0) + offset, tri_index.at(1) + offset});
        index_4d.push_back({tri_index.at(2), tri_index.at(0) + offset, tri_index.at(1) + offset, tri_index.at(2) + offset});
    }

}
#if 0
inline AcceleratedHitManager gltfTetrahedronInBox(size_t dim)
{
    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();
    FloatType target_dist = 3;
    Rotation target_rot(Rotation::fromPlaneAngle(Vec({0,0,0,1}), Vec({0,0,0,1}), M_PI_4));

    if(0){
        //
        // create box
        tinygltf::Model model;
        loadModel(model, "assets/box/box.gltf");

        std::shared_ptr<Mat> vertex_buffer_3d = loadMeshVertices(model, 0);

        std::shared_ptr<Mat> cube_vertex_buffer = std::make_shared<Mat>(Shape({4, vertex_buffer_3d->shape(1) + 1 }));
        cube_vertex_buffer->setBlock(0,0, *vertex_buffer_3d);
        (*cube_vertex_buffer)(cube_vertex_buffer->shape(0)-1, cube_vertex_buffer->shape(1) - 1) = 0.1;

        const size_t FACE_PER_CUBE = 6;

        FloatType face_scale = 0.1;
        std::array<size_t, FACE_PER_CUBE> scale_axis{0,0,1,1,2,2};
        std::array<FloatType, FACE_PER_CUBE> scale_axis_offset{-0.45, 0.45,-0.45, 0.45,-0.45, 0.45};

        size_t last_vertex_idx = cube_vertex_buffer->shape(1) - 1;
        std::vector<std::vector<size_t>> indices = loadMeshIndices(model, 0);
        for(auto & idx: indices) idx.push_back(last_vertex_idx);

        for(size_t face_idx = 0; face_idx < FACE_PER_CUBE; face_idx++)
        {
            std::shared_ptr<Mat> face_vertex_buffer = std::shared_ptr<Mat>(new Mat(*cube_vertex_buffer));
            // (*face_vertex_buffer) = (*cube_vertex_buffer);
            (*face_vertex_buffer)(Row(scale_axis[face_idx])) *= face_scale;
            (*face_vertex_buffer)(Row(scale_axis[face_idx])) += scale_axis_offset[face_idx];

            *face_vertex_buffer = target_rot.apply(*face_vertex_buffer);
            (*face_vertex_buffer)(Row(dim - 1)) += target_dist;

            for(size_t i = 0; i < indices.size(); i++)
            {
                buffer->push_back(Hittable(
                    RigidBody::createPolygonPrimitive(face_vertex_buffer, indices.at(i)),
                    Material::choose(Material::LAMBERTIAN, Pixel({0.3, 0.3, 0.7})),
                    std::string("face_") + std::to_string(face_idx) + "_" + std::to_string(i)));
            }
        }

    }


    if(0){
        //
        // create tetrahedron
        size_t tetrahedron_triangle_num = 4;
        std::shared_ptr<Mat> vertex_buffer = std::shared_ptr<Mat>(
            new Mat({4,5},{
            -.3, 0.4, 0.0, 0.0, 0.0,
            -.3, 0.0, 0.4, 0.0, 0.0,
            -.3, -.3, -.3, 0.4, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1}));

        *vertex_buffer = target_rot.apply(*vertex_buffer);
        (*vertex_buffer)(Row(dim - 1)) += target_dist;

        std::vector<std::vector<size_t>> indices({
            {0,1,2,4},
            {0,1,3,4},
            {0,2,3,4},
            {1,2,3,4}});

        for(size_t i = 0; i < tetrahedron_triangle_num; i++)
        {
            buffer->push_back(Hittable(
                RigidBody::createPolygonPrimitive(vertex_buffer, indices.at(i)),
                Material::choose(Material::LAMBERTIAN, Pixel({0.3, 0.7, 0.3})),
                std::string("tetrahedron_") + std::to_string(i)));
        }
    }

    if(1)
    {
        tinygltf::Model model;
        loadModel(model, "build/assets/Duck.gltf");

        std::shared_ptr<Mat> vertex_buffer_3d = loadMeshAttributes(model, 0, "POSITION");
        (*vertex_buffer_3d)(Row(1)) -= 80;
        (*vertex_buffer_3d) *= 8e-3;

        std::shared_ptr<Mat> vertex_buffer_4d = nullptr;//std::make_shared<Mat>(Shape({4, vertex_buffer_3d->shape(1) + 1}));
        std::vector<std::vector<size_t>> indices_3d = loadMeshIndices(model, 0);
        std::vector<std::vector<size_t>> indices_4d;

        To4DMesh(vertex_buffer_3d, indices_3d, vertex_buffer_4d, indices_4d);

#if 0
        size_t last_vertex_idx = vertex_buffer->shape(1) - 1;
        vertex_buffer->setBlock(0, 0, *vertex_buffer_3d);
        (*vertex_buffer)(Col(vertex_buffer->shape(1) - 1)) = Vec({0,10,0,0.1}).T();
        (*vertex_buffer)(Row(1)) -= 80;
        // (*vertex_buffer)(vertex_buffer->shape(0) - 1, vertex_buffer->shape(1) - 1) = 0.1;

        std::vector<std::vector<size_t>> indices = loadMeshIndices(model, 0);
        for(auto & idx: indices) idx.push_back(last_vertex_idx);
        (*vertex_buffer) *= 2e-3;
        // std::cout << "vertices:" << (*vertex_buffer)(Block({}, {0, 10})).T().str() << std::endl;
#endif
        *vertex_buffer_4d = target_rot.apply(*vertex_buffer_4d);
        (*vertex_buffer_4d)(Row(dim-1)) += target_dist;

        // auto texture = loadMeshTexture(model);

        for(auto & idx: indices_4d)
        {
            buffer->push_back(Hittable(
                RigidBody::createPolygonPrimitive(vertex_buffer_4d, idx),
                Material::choose(Material::LAMBERTIAN,
                // texture.at(idx.at(0)))
                Pixel({0.8, 0.6, 0.3}))
                ));
        }
    }

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;

}
#endif

} // namespace scene
} // namespace rtc
#endif // _SCENES_H_
