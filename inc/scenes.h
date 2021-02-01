#if !defined(_SCENES_H_)
#define _SCENES_H_

#include "material.h"
#include "rigid_body.h"
#include "hittable.h"
#include "bounding_volume_hierarchy.h"
#include "gltf_utils.h"

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

inline AcceleratedHitManager gltf3DBox()
{
    tinygltf::Model model;
    size_t dim = 3;
    loadModel(model, "assets/box/box.gltf");

    std::shared_ptr<Mat> vertex_buffer = loadMeshVertices(model, 0);

    std::vector<std::vector<size_t>> indices = loadMeshIndices(model, 0);

    std::cout << "Vertices: \n" << vertex_buffer->str();

    Rotation r(Rotation::fromPlaneAngle(Vec({1,0,0}), Vec({0,1,1}), 0.9));
    *vertex_buffer = r.apply(*vertex_buffer);
    (*vertex_buffer)(Row(2)) += 3;

    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();
    for(auto & idx: indices)
    {
        buffer->push_back(Hittable(
            RigidBody::createPolygonPrimitive(vertex_buffer, idx),
            Material::choose(Material::LAMBERTIAN, Pixel({0.3, 0.3, 0.6}))));
    }

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;

}

inline AcceleratedHitManager gltf3DSphere()
{
    tinygltf::Model model;
    size_t dim = 3;
    loadModel(model, "assets/sphere/sphere.gltf");

    std::shared_ptr<Mat> vertex_buffer = loadMeshVertices(model, 0);
    // for(size_t i = 0; i < 20; i++)
    // {
    //     std::cout << (*vertex_buffer)(Col(i)).T().str();
    // }

    std::vector<std::vector<size_t>> indices = loadMeshIndices(model, 0);

    // std::cout << "Vertices: \n" << vertex_buffer->str();

    // Rotation r(Rotation::fromPlaneAngle(Vec({1,0,0}), Vec({0,1,1}), 0.));
    // *vertex_buffer = r.apply(*vertex_buffer);
    (*vertex_buffer)(Row(2)) += 3;
    // indices.resize(15);



    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();
    for(auto & idx: indices)
    {
        // for(auto & xyz: idx) std::cout << xyz <<  " ";
        // std::cout << std::endl;
        buffer->push_back(Hittable(
            RigidBody::createPolygonPrimitive(vertex_buffer, idx),
            Material::choose(Material::LAMBERTIAN, Pixel({0.3, 0.3, 0.6}))));
    }

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1, /*verbose*/ false);
    manager.setRoot(root);

    return manager;

}

inline AcceleratedHitManager gltf3DBoomBox()
{
    tinygltf::Model model;
    size_t dim = 3;
    loadModel(model, "build/assets/BoomBox.gltf");

    std::shared_ptr<Mat> vertex_buffer = loadMeshVertices(model, 0);

    std::vector<std::vector<size_t>> indices = loadMeshIndices(model, 0);
    (*vertex_buffer) *= 100.;
    std::cout << "vertices:" << (*vertex_buffer)(Block({}, {0, 10})).T().str() << std::endl;

    Rotation r(Rotation::fromPlaneAngle(Vec({0,1,0}), Vec({1,0,0}), M_PI));
    r = Rotation::fromPlaneAngle(Vec({1,0,0}), Vec({0,0,1}), -M_PI/6) * r;
    *vertex_buffer = r.apply(*vertex_buffer);

    (*vertex_buffer)(Row(2)) += 3;

    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();

    // indices.resize(50);
    for(auto & idx: indices)
    {
        // for(auto & xyz: idx) std::cout << xyz <<  " ";
        // std::cout << std::endl;
        buffer->push_back(Hittable(
            RigidBody::createPolygonPrimitive(vertex_buffer, idx),
            Material::choose(Material::LAMBERTIAN, Pixel({0.3, 0.3, 0.6}))));
    }

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1, /*verbose*/ false);
    manager.setRoot(root);

    return manager;

}

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

    std::shared_ptr<Mat> vertex_buffer_3d = loadMeshVertices(model, 0);
    std::shared_ptr<Mat> vertex_buffer = std::make_shared<Mat>(Shape({4, vertex_buffer_3d->shape(1) + 1}));
    vertex_buffer->setBlock(0,0, *vertex_buffer_3d);
    (*vertex_buffer)(vertex_buffer->shape(0) - 1, vertex_buffer->shape(1) - 1) = .1;

    size_t last_vertex_idx = vertex_buffer->shape(1) - 1;

    std::vector<std::vector<size_t>> indices = loadMeshIndices(model, 0);
    for(auto & idx: indices)
        idx.push_back(last_vertex_idx);

    // std::cout << "Vertices: \n" << vertex_buffer->str();

    Rotation r(Rotation::fromPlaneAngle(Vec({0,0,0,1}), Vec({0,0,0,1}), M_PI_4));
    *vertex_buffer = r.apply(*vertex_buffer);
    (*vertex_buffer)(Row(dim - 1)) += 5;

    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();
    for(auto & idx: indices)
    {
        buffer->push_back(Hittable(
            RigidBody::createPolygonPrimitive(vertex_buffer, idx),
            Material::choose(Material::LAMBERTIAN, Pixel({0.3, 0.3, 0.6}))));
    }

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;

}

inline AcceleratedHitManager gltfTetrahedronInBox(size_t dim)
{
    tinygltf::Model model;
    loadModel(model, "assets/box/box.gltf");

    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();
    FloatType target_dist = 3;
    Rotation target_rot(Rotation::fromPlaneAngle(Vec({0,0,0,1}), Vec({0,0,0,1}), M_PI_4));

    if(1){
        //
        // create box
        std::shared_ptr<Mat> vertex_buffer_3d = loadMeshVertices(model, 0);

        std::shared_ptr<Mat> cube_vertex_buffer = std::make_shared<Mat>(Shape({4, vertex_buffer_3d->shape(1) + 1 }));
        cube_vertex_buffer->setBlock(0,0, *vertex_buffer_3d);
        (*cube_vertex_buffer)(cube_vertex_buffer->shape(0)-1, cube_vertex_buffer->shape(1) - 1) = 0.1;

        // size_t box_vertex_num = vertex_buffer_3d->shape(1);
        // size_t box_triangle_num = 2 * 6;
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


    {
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

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;

}

} // namespace scene
} // namespace rtc
#endif // _SCENES_H_
