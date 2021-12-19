// #include "vec3.h"
#include "mxm/geometry_ray.h"
#include "hittable.h"
#include "mxm/random.h"
#include "mxm/cv_pixel.h"
using namespace mxm;
namespace rtc
{
// const float Material::DEFUALT_FUZZ = -1;

class Lambertian :public Material
{
    public:
        Lambertian(const Pixel& albedo, float_t fuzz):Material(albedo), fuzz_(fuzz == DEFUALT_FUZZ ? 1. : fuzz){}

        virtual Ray scatter(const Ray& ray_in, const RigidBodyHitRecord& record) const override
        {
            Vec target = record.p + record.n + fuzz_ * random::unitSphere<float>(record.p.size());
            return Ray(record.p, target - record.p);
        }

        virtual Ray localFrameScatter(const Ray& ray_in, const RigidBodyHitRecord& record, const RigidTransform<float, 3>& pose) const override
        {
            return scatter(ray_in, record);
        }

    private:
        float_t fuzz_;
};

class Metal :public Material
{
    public:
        Metal(const Pixel& albedo, float_t fuzz):Material(albedo), fuzz_(fuzz == DEFUALT_FUZZ ? 0 : fuzz){}

        virtual Ray scatter(const Ray& ray_in, const RigidBodyHitRecord& record) const
        {
            Vec reflected = reflect(ray_in.direction(), record.n);
            return Ray(record.p, reflected + fuzz_ * random::unitSphere<float>(record.p.size()));
        }
    private:
        float_t fuzz_;
};

class Dielectric :public Material
{
    public:
        Dielectric(float_t ri=1.5):Material(Pixel::white()), ref_idx_(ri){}

        virtual Ray scatter(const Ray& ray_in, const RigidBodyHitRecord& record) const
        {
            ray_in.checkDimension(record.p.size()).checkDimension(record.n.size());

            Vec outward_normal(record.n.size());
            Vec reflected = reflect(ray_in.direction(), record.n);
            float_t ni_over_nt;

            float_t reflect_prob;
            float_t cosine;
            if (ray_in.direction().dot( record.n) > 0) {
                outward_normal = -record.n;
                ni_over_nt = ref_idx_;
                cosine = ref_idx_ * ray_in.direction().dot(record.n);
            }
            else {
                outward_normal = record.n;
                ni_over_nt = 1.0 / ref_idx_;
                cosine = -ray_in.direction().dot( record.n);
            }
            auto refracted = refract(ray_in.direction(), outward_normal, ni_over_nt);
            if(refracted)
            {
                float_t reflect_prob = schlick(cosine, ref_idx_);
                return Ray(record.p, (random::uniform<float>() < reflect_prob) ? reflected : *refracted);
            }
            else
            {
                return Ray(record.p, reflected);
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

