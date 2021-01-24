#if !defined(_GLTF_UTILS_H)
#define _GLTF_UTILS_H

#include <iostream>
#include "linalg.h"
#include "tiny_gltf.h"
using namespace rtc;

inline bool loadModel(tinygltf::Model& model, const std::string& filename)
{
    tinygltf::TinyGLTF loader;
    std::string        err;
    std::string        warn;

    bool res = loader.LoadASCIIFromFile(&model, &err, &warn, filename);
    // bool res(true);
    if (!warn.empty())
    {
        std::cout << "WARN: " << warn << std::endl;
    }

    if (!err.empty())
    {
        std::cout << "ERR: " << err << std::endl;
    }

    if (!res)
        std::cout << "Failed to load glTF: " << filename << std::endl;
    else
        std::cout << "Loaded glTF: " << filename << std::endl;

    return res;
}

inline void dbgModel(tinygltf::Model& model)
{
    for (auto& mesh : model.meshes)
    {
        std::cout << "mesh : " << mesh.name << std::endl;
        for (auto& primitive : mesh.primitives)
        {
            const tinygltf::Accessor& indexAccessor = model.accessors[primitive.indices];

            std::cout << "indexaccessor: count " << indexAccessor.count << ", type "
                      << indexAccessor.componentType << std::endl;

            tinygltf::Material& mat = model.materials[primitive.material];
            for (auto& mats : mat.values)
            {
                std::cout << "mat : " << mats.first.c_str() << std::endl;
            }

            for (auto& image : model.images)
            {
                std::cout << "image name : " << image.uri << std::endl;
                std::cout << "  size : " << image.image.size() << std::endl;
                std::cout << "  w/h : " << image.width << "/" << image.height
                          << std::endl;
            }

            std::cout << "indices : " << primitive.indices << std::endl;
            std::cout << "mode     : "
                      << "(" << primitive.mode << ")" << std::endl;

            for (auto& attrib : primitive.attributes)
            {
                std::cout << "attribute : " << attrib.first.c_str() << std::endl;
            }
        }
    }
}

inline void printVertices(tinygltf::Model& model)
{
    for (auto& mesh : model.meshes)
    {
        std::cout << "mesh : " << mesh.name << std::endl;
        for (auto& primitive : mesh.primitives)
        {
            const tinygltf::Accessor& accessor = model.accessors[primitive.attributes["POSITION"]];

            const tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
            const tinygltf::Buffer&     buffer     = model.buffers[bufferView.buffer];

            const float* positions = reinterpret_cast<const float*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
            std::cout << "count: " << accessor.count << std::endl;
            for (size_t i = 0; i < accessor.count; ++i)
            {
                // Positions are Vec3 components, so for each vec3 stride, offset for x, y, and z.
                std::cout << "(" << positions[i * 3 + 0] << ", " // x
                          << positions[i * 3 + 1] << ", "        // y
                          << positions[i * 3 + 2] << ")"         // z
                          << "\n";
            }
        }
    }
}

inline std::vector<std::vector<size_t>> loadMeshIndices(
    tinygltf::Model& model,
    size_t mesh_idx)
{
    std::vector<std::vector<size_t>> ret;
    auto & mesh(model.meshes.at(mesh_idx));
    std::cout << "mesh : " << mesh.name << std::endl;
    auto& primitive = mesh.primitives.at(0);

    if(primitive.mode != TINYGLTF_MODE_TRIANGLES)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    const tinygltf::Accessor& accessor = model.accessors[primitive.indices];;

    if(accessor.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    const tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
    const tinygltf::Buffer&     buffer     = model.buffers[bufferView.buffer];

    const uint16_t* indices = reinterpret_cast<const uint16_t*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);

    for (size_t i = 0; i < accessor.count; i+=3)
    {
        ret.push_back({indices[i], indices[i+1], indices[i+2]});
    }
    return ret;
}

inline std::shared_ptr<rtc::Mat> loadMeshVertices(
    tinygltf::Model& model,
    size_t mesh_idx)
{
    std::shared_ptr<rtc::Mat> ret;
    auto & mesh(model.meshes.at(mesh_idx));
    std::cout << "mesh : " << mesh.name << std::endl;
    auto& primitive = mesh.primitives.at(0);

    if(primitive.mode != TINYGLTF_MODE_TRIANGLES)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));


    const tinygltf::Accessor& accessor = model.accessors[primitive.attributes["POSITION"]];

    const tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
    const tinygltf::Buffer&     buffer     = model.buffers[bufferView.buffer];

    const float* positions = reinterpret_cast<const float*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);

    ret = std::make_shared<Mat>(Shape({3, accessor.count}));

    for (size_t i = 0; i < accessor.count; ++i)
    {
        (*ret)(Col(i)) = Vec({positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]});
    }
    return ret;

}

#endif // _GLTF_UTILS_H
