#if !defined(_SCENE_4D_H_)
#define _SCENE_4D_H_

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
inline AcceleratedHitManager simple4D_002()
{
    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();

    size_t dim = 4;

    buffer->push_back(Hittable(
        RigidBody::choose(RigidBody::SPHERE, Vec({0,0,0,4}), Rotation::Identity(dim), {1}),
        Material::choose(Material::METAL), "iron ball"));

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

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
    const RigidTrans& pose, const std::string& gltf_path, FloatType height=1., FloatType scale=1.0)
{
    tinygltf::Model model;
    size_t dim = 4;
    loadModel(model, gltf_path);

    std::shared_ptr<Mat> vertex_buffer_3d = loadMeshAttributes(model, 0, "POSITION");
    (*vertex_buffer_3d) *= scale;
    auto vertex_index_buffer_3d = std::make_shared<Matrix<size_t>>(loadMeshIndices(model, 0));

    return Hittable(
        RigidBody::createPrism(pose.translation(), pose.rotation(), height, vertex_buffer_3d, vertex_index_buffer_3d),
        Material::choose(Material::METAL, Pixel({0.1, 0.1, 0.1})));
}

inline Hittable loadTexturelessCavityPrism4D(
    const RigidTrans& pose, const std::string& gltf_path, FloatType height=1.,FloatType thick_rate=0.9)
{
    tinygltf::Model model;
    size_t dim = 4;
    loadModel(model, gltf_path);

    auto vertex_buffer_out = *loadMeshAttributes(model, 0, "POSITION");
    auto vertex_buffer_in = vertex_buffer_out * thick_rate;
    auto vertex_buffer_3d = std::make_shared<Mat>(hstack(vertex_buffer_out, vertex_buffer_in));

    auto vertex_index_buffer_out = loadMeshIndices(model, 0);
    auto vertex_index_buffer_in = vertex_index_buffer_out + vertex_buffer_out.shape(1);
    auto vertex_index_buffer_3d =
        std::make_shared<Matrix<size_t>>(hstack(vertex_index_buffer_out, vertex_index_buffer_in));

    return Hittable(
        RigidBody::createPrism(pose.translation(), pose.rotation(), height, vertex_buffer_3d, vertex_index_buffer_3d),
        Material::choose(Material::LAMBERTIAN, Pixel({0.3, 0.3, 0.2})));
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


inline AcceleratedHitManager gltfDuckInBox4D()
{
    size_t dim = 4;
    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();

    RigidTrans common_pose(Vec({0,0,0,2}), Rotation::fromPlaneAngle(Vec({1,.5,0,0}), Vec({0,0,0,1}), .5*M_PI_4));

    buffer->push_back(loadTexturelessCavityPrism4D(
        common_pose, "assets/box/box.gltf", 0.01));

    RigidTrans duck_pose(Vec({0.1, 0.475, 0, 0}),
        // Rotation::fromPlaneAngle(Vec({1,0,0,0}), Vec({0,0,1,0}), M_PI_2)*
        Rotation::fromPlaneAngle(Vec({1,0,0,0}), Vec({0,1,0,0}), M_PI));

    buffer->push_back(loadPrism4D(
        common_pose * duck_pose, "build/assets/Duck.gltf", 4e-3, 0.01));

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;
}

inline AcceleratedHitManager gltfBoxFrame4D()
{
    size_t dim = 4;
    HittableBufferPtr buffer = std::make_shared<HittableBuffer>();

    FloatType frame_width = 0.1;

    FloatType cube_h = 1.;

    RigidTrans global_pose(Vec({0,0,0,3}), Rotation::fromPlaneAngle(Vec({1,1,0,0}),Vec({0,0,0,1}), 0.9*M_PI_4));

    Mat offset_table({4,8},{
    0,0,0,0,
    0,0,1,0,
    0,1,0,0,
    0,1,1,0,
    1,0,0,0,
    1,0,1,0,
    1,1,0,0,
    1,1,1,0}, Mat::COL);
    offset_table(Block({0,end()-1},{})) *= cube_h;
    offset_table(Block({0,end()-1},{})) -= cube_h*.5;

    std::cout << offset_table.str() << std::endl;

    std::vector<Rotation> rots{
        Rotation::Identity(dim),
        Rotation::fromPlaneAngle(Vec({1,0,0,0}),Vec({0,0,0,1}), M_PI_2),
        Rotation::fromPlaneAngle(Vec({0,1,0,0}),Vec({0,0,0,1}), M_PI_2),
        Rotation::fromPlaneAngle(Vec({0,0,1,0}),Vec({0,0,0,1}), M_PI_2)};

    auto frame_tf = RigidTrans(Vec({-.8,0,0,0}));

    for(size_t axis = 0; axis < dim; axis++)
    {
        for(uint8_t i = 0; i < 8; i++)
        {
            auto tf = RigidTrans(offset_table(Col(i)).asVector());
            tf = RigidTrans(rots.at(axis)) * tf;
            buffer->push_back(loadTexturelessPrism4D(
                global_pose * frame_tf * tf,
                "assets/box/box.gltf", cube_h, frame_width));
        }
    }

    auto sphere_tf = global_pose * RigidTrans(Vec({1.2,0,0,0}));

    buffer->push_back(Hittable(
        RigidBody::choose(RigidBody::SPHERE, sphere_tf.translation(), sphere_tf.rotation(), {1}),
        Material::choose(Material::METAL, Pixel({0.5, 0.5, 0.5})), "black ball"));

    AcceleratedHitManager manager;
    auto root = std::shared_ptr<bvh::Node>(new bvh::Node(dim, buffer, {0, buffer->size()}));
    root->split(1);
    manager.setRoot(root);

    return manager;
}

} // namespace scene
} // namespace rtc


#endif // _SCENE_4D_H_
