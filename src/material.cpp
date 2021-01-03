// #include "vec3.h"
#include "ray.h"
#include "hittable.h"
#include "random_factory.h"

namespace rtc
{
const float Material::DEFUALT_FUZZ = -1;

class Lambertian :public Material
{
    public:
        Lambertian(const Pixel& albedo, float_t fuzz):Material(albedo), fuzz_(fuzz == DEFUALT_FUZZ ? 1. : fuzz){}

        virtual Ray scatter(const Ray& ray_in, const Vec& hit_p, const Vec& hit_n) const
        {
            Vec target = hit_p + hit_n + fuzz_ * random::unitSphere();
            return Ray(hit_p, target - hit_p);
        }

    private:
        float_t fuzz_;
};

class Metal :public Material
{
    public:
        Metal(const Vec& albedo, float_t fuzz):Material(albedo), fuzz_(fuzz == DEFUALT_FUZZ ? 0 : fuzz){}

        virtual Ray scatter(const Ray& ray_in, const Vec& hit_p, const Vec& hit_n) const
        {
            Vec reflected = reflect(ray_in.direction(), hit_n);
            return Ray(hit_p, reflected + fuzz_ * random::unitSphere());
        }
    private:
        float_t fuzz_;
};

class Dielectric :public Material
{
    public:
        Dielectric(float_t ri=1.5):Material(Pixel::white()), ref_idx_(ri){}

        virtual Ray scatter(const Ray& ray_in, const Vec& hit_p, const Vec& hit_n) const
        {
            ray_in.checkDimension(hit_p.size()).checkDimension(hit_n.size());

            Vec outward_normal(hit_n.size());
            Vec reflected = reflect(ray_in.direction(), hit_n);
            float_t ni_over_nt;

            float_t reflect_prob;
            float_t cosine;
            if (ray_in.direction().dot( hit_n) > 0) {
                outward_normal = -hit_n;
                ni_over_nt = ref_idx_;
                cosine = ref_idx_ * ray_in.direction().dot(hit_n);
            }
            else {
                outward_normal = hit_n;
                ni_over_nt = 1.0 / ref_idx_;
                cosine = -ray_in.direction().dot( hit_n);
            }
            auto refracted = refract(ray_in.direction(), outward_normal, ni_over_nt);
            if(refracted)
            {
                float_t reflect_prob = schlick(cosine, ref_idx_);
                return Ray(hit_p, (random::UnitFloat() < reflect_prob) ? reflected : *refracted);
            }
            else
            {
                return Ray(hit_p, reflected);
            }
        }
    private:
        float_t ref_idx_;
};

MaterialPtr Material::choose(Types type, const Pixel& albedo, float_t fuzz)
{
    if(type == Material::METAL)
        return std::make_shared<Metal>(albedo, fuzz);
    else if (type == Material::LAMBERTIAN)
        return std::make_shared<Lambertian>(albedo, fuzz);
    else if (type == Material::DIELECTRIC)
        return std::make_shared<Dielectric>();

    return std::make_shared<Metal>(albedo, fuzz);
}

} // namespace rtc

