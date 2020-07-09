#include "ray.h"
#include "io.h"
#include <vector>
#include <iostream>

using namespace rtc;

bool hit_sphere(const Vector3& center, float_t radius, const Ray& r) {
    auto oc = center - r.origin();
    auto unit_dir = r.direction();
    float dist = oc.cross(unit_dir).norm();
    return (dist < radius);

    // Vector3 oc = r.origin() - center;
    // float a = r.direction().dot( r.direction());
    // float b = 2.0 * oc.dot( r.direction());
    // float c = oc.dot( oc) - radius*radius;
    // float discriminant = b*b - 4*a*c;
    // return (discriminant > 0);
}

Pixel color(const Ray& ray)
{
    if (hit_sphere(Pixel{1,0,-1}, 0.5, ray))
        return Pixel{0, 0, 0};
    Vector3 direction(ray.direction());
    float_t t = 0.5 * (direction.y() + 1.);
    return (1- t) * Vector3{1,1,1} + t * Vector3{0.5, 0.7, 1.};
}

int main(int argc, char const *argv[])
{
    int nx = 1000;
    int ny = 500;
    Vector3 lower_left_corner{-2.0, -1.0, -1.0};
    Vector3 horizontal{4.0, 0.0, 0.0};
    Vector3 vertical{0.0, 2.0, 0.0};
    Vector3 origin{0.0, 0.0, 0.0};
    std::vector<Pixel> img;
    
    for (int j = ny-1; j >= 0; j--) {
        for (int i = 0; i < nx; i++) {
            float u = float(i) / float(nx);
            float v = float(j) / float(ny);
            Ray r(origin, lower_left_corner + u*horizontal + v*vertical);
            img.push_back(color(r));
        }
    }
    writeToPPM("exercise_4.ppm", nx, ny, img);
    return 0;
}
