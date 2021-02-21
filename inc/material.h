#if !defined(_MATERIAL_H_)
#define _MATERIAL_H_

// #include "vec3.h"
#include "pixel.h"
#include "linalg.h"
#include "ray.h"
#include "random_factory.h"
#include <memory>
#include "primitive_geometry.h"
#include "interpolation.h"
#include <iostream>
#include "rigid_body.h"


namespace rtc
{
// class HitRecord;
class Material
{
public:
    Material():albedo_(Pixel::black()){}
    Material(const Pixel& albedo):albedo_(albedo){}
    Material(const Pixel::InitialType& albedo):albedo_(albedo){}

    // const Pixel& attenuation() const { return albedo_; }
    // virtual const Pixel& attenuation(const Vec& hit_p) const
    virtual const Pixel& attenuation(const RigidBody::HitRecord& record) const
    {
        return albedo_;
    }

    virtual Ray scatter(
        const Ray& ray_in, const Vec& hit_p, const Vec& hit_n) const = 0;

    enum Types
    {
        METAL,
        LAMBERTIAN,
        DIELECTRIC
    };
    static const float_t DEFUALT_FUZZ;
    static std::shared_ptr<Material> choose(Types type, const Pixel& albedo=Pixel({.8, .8, .8}), float_t fuzz=DEFUALT_FUZZ);

private:
    Pixel albedo_;
};
using MaterialPtr = std::shared_ptr<Material>;

inline Vec reflect(VecIn in, const UnitVec& norm)
{ return in - norm * 2 * in.dot(norm); }

inline std::shared_ptr<Vec> refract(const UnitVec& dir_in, const UnitVec& normal, float_t ni_over_nt)
{
    float_t dt = dir_in.dot(normal);
    float discriminant = 1.0 - ni_over_nt*ni_over_nt*(1-dt*dt);
    if (discriminant <= 0) return nullptr;
    return std::make_shared<Vec>(ni_over_nt * (dir_in - normal * dt) - normal * sqrt(discriminant));
}

inline float_t schlick(float_t cosine, float_t ref_idx) {
    float_t r0 = (1-ref_idx) / (1+ref_idx);
    r0 = r0*r0;
    return r0 + (1-r0)*pow((1 - cosine),5);
}

struct TextureBuffer
{
    // shape = {2, vertex_num}
    Mat tex_coord;

    // shape = {h, w}
    Matrix<Pixel> base_texture;
    Mat normal;
    // shape = {h, w}
    // Matrix<Pixel> metallic_roughness_texture;
};

using TextureBufferPtr = std::shared_ptr<TextureBuffer>;

class GltfTexture :public Material
{
    public:
        GltfTexture(
            TextureBufferPtr& tex_buffer,
            std::shared_ptr<Matrix<size_t>>& indices,
            std::shared_ptr<Mat>& vertex_buffer)
            :tex_buffer_(tex_buffer),
            vertex_indices_(indices),
            vertex_buffer_(vertex_buffer) {}

        virtual Ray scatter(const Ray& ray_in, const Vec& hit_p, const Vec& hit_n) const override
        {
            return Ray(hit_p, reflect(ray_in.direction(), hit_n) + 0. * random::unitSphere(hit_p.size()));
        }

        virtual const Pixel& attenuation(const RigidBody::HitRecord& record) const override
        {
            if(tex_buffer_->base_texture.shape() == Shape({1,1}))
                return tex_buffer_->base_texture(0,0);
            if(record.prim_idx >= vertex_indices_->shape(1))
                return tex_buffer_->base_texture(0,0);
            // Triangle A B C, put A to (0,0), put B to (AB.norm(), 0)

            Mat triangle = getPrimitive(*vertex_buffer_, *vertex_indices_, record.prim_idx);
            Mat tex_coord = getPrimitive(tex_buffer_->tex_coord, *vertex_indices_, record.prim_idx);

            Mat triangle_2d;
            Vec hit_p_2d;
            Vec hit_p_3d({record.prim_coord_hit_p(0), record.prim_coord_hit_p(1), record.prim_coord_hit_p(2)});
            putTriangleInPlane(triangle, hit_p_3d, triangle_2d, hit_p_2d);

            auto hit_p_tex_coord = interp::triangular(hit_p_2d, triangle_2d, tex_coord);
            if(hit_p_tex_coord(1,0) > 1. || hit_p_tex_coord(1,0) < 0 ||
                hit_p_tex_coord(0,0) > 1. || hit_p_tex_coord(0,0) < 0)
            {
                std::cout << "============== debug info ==============\n"
                << "hit_p_tex_coord: " << hit_p_tex_coord.T().str()
                << "prim idx: " << record.prim_idx << "/" << vertex_indices_->shape(1) << "\n"
                << "triangle: \n" << triangle.str() << "\n"
                << "prim_coord hit p: \n" << record.prim_coord_hit_p.str() << "\n"
                << "tri_2d: \n" << triangle_2d.str() << "\n"
                << "hit_p_2d: \n" << hit_p_2d.str() << "\n";
                throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
            }
            // auto hit_p_tex_coord = tex_coord(Col(0));
            size_t u = hit_p_tex_coord(1,0) * tex_buffer_->base_texture.shape(1);
            size_t v = hit_p_tex_coord(0,0) * tex_buffer_->base_texture.shape(0);
            // std::cout << "u: " << u << ", v: " << v << std::endl;
            return tex_buffer_->base_texture(u, v);
        }
    private:
        TextureBufferPtr tex_buffer_;
        std::shared_ptr<Matrix<size_t>> vertex_indices_;
        std::shared_ptr<Mat> vertex_buffer_;
};


} // namespace rtc
#endif // _MATERIAL_H_
