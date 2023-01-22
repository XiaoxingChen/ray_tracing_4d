#include "io.h"
#include <vector>
#include <iostream>
#include "mxm/model_camera.h"
#include "mxm/random.h"
#include "ThreadPool.h"
#include "material.h"
#include "rigid_body.h"
#include "hittable.h"
#include "ray_tracer.h"
#include "scenes.h"
#include "utils.h"
#include "mxm/geometry_cube.h"
#include "mxm/geometry_simplex.h"

using namespace rtc;
using namespace mxm;

const size_t DIM = 3;

template<typename DType, size_t DIM>
Vector<DType> vertexZBuffer(const Camera<DType, DIM>& cam, const Matrix<DType>& points)
{
    Matrix<DType> body_frame_points = cam.pose().inv().apply(points);
    return body_frame_points(Row(end() - 1));
}

template<typename DType>
DType orienDir(const Matrix<DType>& prim)
{
    Matrix<DType> mat({prim.shape(0), prim.shape(1)-1});
    for(size_t i = 0; i < mat.shape(1); i++)
    {
        mat(Col(i)) = prim(Col(i+1)) - prim(Col(i));
    }
    return det(mat);
}

void fragmentShading(
    const Matrix<float>& vertex_buffer, 
    const Matrix<size_t>& index_buffer, 
    const Matrix<Pixel>& texture,
    const Matrix<float>& texture_coord_buffer,
    const Vector<float>& vertex_z_buffer,
    Matrix<float>& z_buffer,
    Matrix<Pixel>& output_img)
{
    Matrix<float> prim({2,3});
    AABB<float> prim_aabb(2);
    Matrix<float> prim_texture_coord({2,3});
    AABB<int> screen_aabb({0,0}, {int(output_img.shape(0)) - 1, int(output_img.shape(1)) - 1});
    for(size_t prim_idx = 0; prim_idx < index_buffer.shape(1); prim_idx++)
    {
        prim_aabb.clear();
        bool valid_z = false;
        Vector<float> vertex_z(3);
        for(size_t i = 0; i < 3; i++)
        {
            size_t vtx_idx = index_buffer(i, prim_idx);
            Matrix<float> vertex = vertex_buffer(Col(vtx_idx));
            prim_aabb.extend(vertex);
            prim(Col(i)) = vertex;
            prim_texture_coord(Col(i)) = texture_coord_buffer(Col(vtx_idx));
            
            if(screen_aabb.contains(vertex)(0))
            {
                valid_z |= vertex_z_buffer(vtx_idx) <= interp::bilinearUnitSquare(vertex, z_buffer, "nearest")(0,0);
            }
            vertex_z(i) = vertex_z_buffer(vtx_idx);
        }
        if(! valid_z) continue; // depth test
        if(orienDir(prim) < 0) continue; // face culling
        Vector<float> extent = prim_aabb.max() - prim_aabb.min();
        std::vector<float> sample_pts_data;
        sample_pts_data.reserve(size_t(extent(0) * extent(1) + 1));
        
        for(size_t i = size_t(prim_aabb.min()(0)); i < prim_aabb.max()(0) + 1; i++)
        {
            for(size_t j = size_t(prim_aabb.min()(1)); j < prim_aabb.max()(1) + 1; j++)
            {
                sample_pts_data.push_back(float(i));
                sample_pts_data.push_back(float(j));
            }
        }

        // std::cout << "extent: " <<  sample_pts_data.size() << std::endl;
        Matrix<float> sample_pts(fixRow(2), std::move(sample_pts_data), COL);
        auto local_inside = splx::arePointsInside(sample_pts, prim, float(1e-2));
        
        auto bary_coord = splx::barycentricCoordinate(sample_pts, prim);
        // Pixel px_color = bary_coord.T().matmul(colors); // todo
        auto texture_coord = prim_texture_coord.matmul(bary_coord);

        // std::cout << mxm::to_string(texture_coord) << std::endl;

        auto colors = interp::bilinearUnitSquare(texture_coord, texture);
        Vector<float> px_z = bary_coord.T().matmul(vertex_z);

        // std::cout << mxm::to_string(color) << std::endl;
        
        for(size_t i = 0; i < sample_pts.shape(1); i++)
        {
            Vector<int> px_coord(sample_pts(Col(i)));
            if(!screen_aabb.contains(px_coord)(0)) continue;
            if(local_inside(i) == 0) continue;
            
            
            output_img(px_coord(0), px_coord(1)) = colors(i, 0);
            z_buffer(px_coord(0), px_coord(1)) = px_z(i);
        }
        // std::cout << "inside count: " << inside_cnt << std::endl;
    }
}

int main(int argc, char const *argv[])
{
    Camera<float, DIM> cam;
    cam.setFocalLength({250, 250}).setResolution({240, 320}).setPrincipalOffset({125, 125});

    Rotation<float> rot = Rotation<float>::fromAxisAngle({1.f, 0,0}, M_PI * 0.2);

    Shape output_reso{cam.resolution(0), cam.resolution(1)};
    Vector<size_t> ncube_resolution{5,5,0};

    

    Matrix<Pixel> texture({2,2}, {Pixel::red(), Pixel::green(), Pixel::blue(), Pixel::white()});
    

    auto vertices_3d = generateNCubeVertices({100.f, 100.f, 1.f}, ncube_resolution);

    vertices_3d = rot.apply(vertices_3d);

    vertices_3d += Vector<float>{-50,-50,100};
    Vector<float> vertex_z_buffer = vertexZBuffer(cam, vertices_3d);


    Matrix<float> texture_coord_buffer = random::uniform<float>({2, vertices_3d.shape(1)});

    auto indices = generateSquareTriangleIndices(ncube_resolution);

    Matrix<Pixel> img_out(output_reso);
    Matrix<float> z_buffer = Matrix<float>::ones(img_out.shape()) * std::numeric_limits<float>::max();
    img_out *= 0.;

    // auto vertices = cam.project(vertices_3d);
    auto vertices_3d2 = vertices_3d + Vector<float>{0,50,0};
    fragmentShading(cam.project(vertices_3d), indices, texture, texture_coord_buffer, vertex_z_buffer,z_buffer, img_out);
    fragmentShading(cam.project(vertices_3d2), indices, texture, texture_coord_buffer, vertex_z_buffer,z_buffer, img_out);

    img_out *= 255.;

    imwrite("build/test_img.png", img_out);
    imwrite("build/zbuffer.png", z_buffer);
}