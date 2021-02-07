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
    virtual const Pixel& attenuation(const Vec& hit_p) const
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
    // shape = {h, w}
    // Matrix<Pixel> metallic_roughness_texture;
};

using TextureBufferPtr = std::shared_ptr<TextureBuffer>;

class GltfTexture :public Material
{
    public:
        GltfTexture(
            TextureBufferPtr& tex_buffer,
            const std::vector<size_t>& indices,
            std::shared_ptr<Mat>& vertex_buffer)
            :tex_buffer_(tex_buffer),
            vertex_indices_(indices),
            vertex_buffer_(vertex_buffer) {}

        virtual Ray scatter(const Ray& ray_in, const Vec& hit_p, const Vec& hit_n) const override
        {
            return Ray(hit_p, hit_n + 0.3 * random::unitSphere(hit_p.size()));
        }

        virtual const Pixel& attenuation(const Vec& hit_p) const override
        {
            // Triangle A B C, put A to (0,0), put B to (AB.norm(), 0)
            Mat triangle({hit_p.size(), 3});
            Mat tex_coord({2,3});
            for(size_t i = 0; i < 3; i++)
            {
                triangle(Col(i)) = (*vertex_buffer_)(Col(vertex_indices_.at(i)));
                tex_coord(Col(i)) = tex_buffer_->tex_coord(Col(vertex_indices_.at(i)));
            }

            Mat triangle_2d;
            Vec hit_p_2d;
            putTriangleInPlane(triangle, hit_p, triangle_2d, hit_p_2d);

            auto hit_p_tex_coord = interp::triangular(hit_p_2d, triangle_2d, tex_coord);
            // auto hit_p_tex_coord = tex_coord(Col(0));
            size_t u = hit_p_tex_coord(1,0) * tex_buffer_->base_texture.shape(1);
            size_t v = hit_p_tex_coord(0,0) * tex_buffer_->base_texture.shape(0);
            // std::cout << "u: " << u << ", v: " << v << std::endl;
            return tex_buffer_->base_texture(u, v);
        }
    private:
        TextureBufferPtr tex_buffer_;
        std::vector<size_t> vertex_indices_;
        std::shared_ptr<Mat> vertex_buffer_;
};


} // namespace rtc
#endif // _MATERIAL_H_
