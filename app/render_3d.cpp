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


using namespace rtc;
using namespace mxm;

const size_t DIM = 3;

int main(int argc, char const *argv[])
{
    Camera<float, DIM> cam;//(Vec(3), Rotation::fromAxisAngle(Vec({0,0,1}), 0), Vec({500, 500}), Vec({640, 480}));
    // cam.setFocalLength({500, 500}).setResolution({640, 480});
    cam.setFocalLength({250, 250}).setResolution({320, 240}).setPrincipalOffset({125, 125});
    // Camera cam(Vec(3), Rotation::fromAxisAngle(Vec({0,0,1}), 0), Vec({250, 250}), Vec({320, 240}));
    // auto scene = scene::rectangle3D_002();
    // auto scene = scene::gltf3DBox();
    // auto scene = scene::gltf3DSphere();
    // auto scene = scene::gltf3DDroplet();
    // auto scene = scene::simple3D_005();
    // auto scene = scene::gltf3DBoomBox();
    auto scene = scene::gltf3DDuck();
    // auto scene = scene::simple3DPrism();
    // auto scene = scene::gltfDuckInBox3D();

    auto render = RenderSample<DIM>();
    render.setOutputFilename("image_3d");
    render.setCamera(cam)
        .setMode(RenderSample<DIM>::eMULTITHREADING)
        // .setMode(RenderSample<DIM>::eSINGLE_RAY)
        // .setMode(RenderSample<DIM>::eNAIVE)
        .setRecursionDepth(3)
        .enableRayStack(true)
        .setSampleNum(4)
        .setScene(&scene)
        // .setTestRay(cam.pixelRay({320, 240}))
        .setTestRay(cam.pixelRay({269, 129}))
        // .setTestRay(Ray(Vec({-0.032410,0.209743,2.759068}), Vec({0.997053,0.075776,-0.011965})))
        .run();

    return 0;
}
