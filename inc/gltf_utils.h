#if !defined(_GLTF_UTILS_H)
#define _GLTF_UTILS_H

#include <iostream>
#include <map>
#include "mxm/linalg.h"
#include "mxm/cv_pixel.h"
#include "tiny_gltf.h"
using namespace rtc;
using namespace mxm;

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

inline Matrix<size_t> loadMeshIndices(
    tinygltf::Model& model,
    size_t mesh_idx)
{

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

    Matrix<size_t> ret({3, accessor.count / 3});
    for (size_t i = 0; i < accessor.count; i+=3)
    {
        // ret.push_back({indices[i], indices[i+1], indices[i+2]});
        ret(0, i / 3) = indices[i];
        ret(1, i / 3) = indices[i + 1];
        ret(2, i / 3) = indices[i + 2];
    }
    return ret;
}

inline std::shared_ptr<mxm::Mat> loadMeshVertices(
    tinygltf::Model& model,
    size_t mesh_idx)
{
    std::cout << "WARNING: function deprecated, use loadMeshAttributes instead! " << std::endl;
    std::shared_ptr<mxm::Mat> ret;
    auto & mesh(model.meshes.at(mesh_idx));
    std::cout << "mesh : " << mesh.name << std::endl;
    auto& primitive = mesh.primitives.at(0);

    if(primitive.mode != TINYGLTF_MODE_TRIANGLES)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));


    const tinygltf::Accessor& accessor = model.accessors[primitive.attributes["POSITION"]];

    if(accessor.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    const tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
    if(bufferView.target != TINYGLTF_TARGET_ARRAY_BUFFER)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    const tinygltf::Buffer&     buffer     = model.buffers[bufferView.buffer];

    // const float* positions = reinterpret_cast<const float*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
    // std::cout << "bufferView.byteStride: " << bufferView.byteStride << std::endl;
    // std::cout << "accessor.byteOffset: " << accessor.byteOffset << std::endl;

    ret = std::make_shared<Mat>(Shape({3, accessor.count}));
    size_t byteStride = bufferView.byteStride > 0 ? bufferView.byteStride : 3 * sizeof(float);

    for (size_t i = 0; i < accessor.count; ++i)
    {
        const float* positions = reinterpret_cast<const float*>(&buffer.data[bufferView.byteOffset + i * byteStride + accessor.byteOffset]);
        (*ret)(Col(i)) = Vec({positions[0], positions[1], positions[2]});
    }
    return ret;

}

inline std::shared_ptr<mxm::Mat> loadMeshAttributes(
    tinygltf::Model& model,
    size_t mesh_idx,
    const std::string& type)
{

    std::shared_ptr<mxm::Mat> ret;
    auto & mesh(model.meshes.at(mesh_idx));
    std::cout << "load " + type << std::endl;
    auto& primitive = mesh.primitives.at(0);

    if(primitive.attributes.count(type) == 0)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    if(primitive.mode != TINYGLTF_MODE_TRIANGLES)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));


    const tinygltf::Accessor& accessor = model.accessors[primitive.attributes[type]];

    if(accessor.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    const tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
    if(bufferView.target != TINYGLTF_TARGET_ARRAY_BUFFER)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    const tinygltf::Buffer&     buffer     = model.buffers[bufferView.buffer];

    // const float* positions = reinterpret_cast<const float*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
    // std::cout << "bufferView.byteStride: " << bufferView.byteStride << std::endl;
    // std::cout << "accessor.byteOffset: " << accessor.byteOffset << std::endl;

    size_t data_width = accessor.type == TINYGLTF_TYPE_VEC3 ? 3
        : accessor.type == TINYGLTF_TYPE_VEC2 ? 2 : -1;

    ret = std::make_shared<Mat>(Shape({data_width, accessor.count}));
    size_t byteStride = bufferView.byteStride > 0 ? bufferView.byteStride : data_width * sizeof(float);

    for (size_t i = 0; i < accessor.count; ++i)
    {
        const float* local_addr = reinterpret_cast<const float*>(&buffer.data[bufferView.byteOffset + i * byteStride + accessor.byteOffset]);
        for(size_t j = 0; j < data_width; j++)
            (*ret)(j,i) = local_addr[j];
    }
    return ret;

}

inline std::shared_ptr<Matrix<Pixel>> loadMeshTexture(
    tinygltf::Model& model)
{
    if (model.textures.size() == 0)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    tinygltf::Texture &tex = model.textures[0];

    if (tex.source < 0)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    tinygltf::Image &image = model.images[tex.source];

    if (tex.source < 0)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    if(image.component != TINYGLTF_TYPE_VEC3 && image.component != TINYGLTF_TYPE_VEC4)
    {
        std::cout << "image.component: " << image.component << std::endl;
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    size_t pixel_stride =
        image.component == TINYGLTF_TYPE_VEC3 ? 3 :
        image.component == TINYGLTF_TYPE_VEC4 ? 4 : 0;

    std::cout << "pixel channel: " << pixel_stride
    << ", image w: " << image.width << ", h: " << image.height
    << std::endl;

    if (image.bits != 8)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    auto ret = std::shared_ptr<Matrix<Pixel>>(new Matrix<Pixel>({(size_t)image.height, (size_t)image.width}));
    ret->traverse([&](size_t i, size_t j){
        for(size_t k = 0; k < 3; k++)
            (*ret)(i,j)(k) = image.image.at((i * image.width + j) * pixel_stride + k) / 255.;
        });
#if 0
    auto p_tex_coord = loadMeshAttributes(model, 0, "TEXCOORD_0");
    auto ret = std::vector<rtc::Pixel>(p_tex_coord->shape(1));

    for(size_t i = 0; i < p_tex_coord->shape(1); i++)
    {
        size_t idx_u = (*p_tex_coord)(0, i) * image.width;
        size_t idx_v = (*p_tex_coord)(1, i) * image.height;
        size_t start_i = (image.width * idx_v + idx_u) * pixel_stride;
        for(size_t j = 0; j < rtc::Pixel::size(); j++)
            ret.at(i)(j) = image.image.at(start_i + j) / 255.;
    }
#endif
    std::cout << "Texture loading finished..." << std::endl;

    // for(size_t i = 0; i < 10; i++)
    // {
    //     std::cout << ret.at(i).str() << "\n";
    // }

    // float blue_sum = 0;
    // for(auto & px: ret) blue_sum += px(2);
    // std::cout << "blue sum: " << blue_sum << "\n";

    return ret;
}

#endif // _GLTF_UTILS_H
