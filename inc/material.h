#if !defined(_MATERIAL_H_)
#define _MATERIAL_H_

#include "vec3.h"
#include "ray.h"
#include "random_factory.h"

namespace rtc
{
// class HitRecord;
class Material
{
    public:

        Material(const Vector3& albedo=V3()):albedo_(albedo){}

        const Pixel& attenuation() const { return albedo_; }
        virtual Ray scatter(
            const Ray& ray_in, const Vector3& hit_p, const Vector3& hit_n) const = 0;

        enum Types
        {
            METAL,
            LAMBERTIAN,
            DIELECTRIC
        };
        static const float_t DEFUALT_FUZZ;
        static std::shared_ptr<Material> choose(Types type, const Pixel& albedo=V3{.8, .8, .8}, float_t fuzz=DEFUALT_FUZZ);

    private:
        Pixel albedo_;
};
using MaterialPtr = std::shared_ptr<Material>;

inline Vector3 reflect(V3in in, V3in norm)
{ return in - 2 * in.dot(norm) * norm; }

inline std::shared_ptr<V3> refract(const UnitVector3& dir_in, const UnitVector3& normal, float_t ni_over_nt)
{
    float_t dt = dir_in.dot(normal);
    float discriminant = 1.0 - ni_over_nt*ni_over_nt*(1-dt*dt);
    if (discriminant <= 0) return nullptr;
    return std::make_shared<V3>(ni_over_nt * (dir_in - normal * dt) - normal * sqrt(discriminant));
}

inline float_t schlick(float_t cosine, float_t ref_idx) {
    float_t r0 = (1-ref_idx) / (1+ref_idx);
    r0 = r0*r0;
    return r0 + (1-r0)*pow((1 - cosine),5);
}


} // namespace rtc
#endif // _MATERIAL_H_
