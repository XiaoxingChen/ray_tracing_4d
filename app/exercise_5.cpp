#include "io.h"
#include <vector>
#include <iostream>
#include "camera.h"
#include "random_factory.h"
#include "ThreadPool.h"
#include "material.h"
#include "rigid_body.h"
#include "hittable.h"
#include "ray_tracer.h"
#include "scenes.h"
#include "utils.h"


using namespace rtc;


int main(int argc, char const *argv[])
{
    Camera cam(Vec(3), Rotation::fromAxisAngle(Vec({0,0,1}), 0), Vec({500, 500}), Vec({640, 480}));
    // Camera cam(Vec(3), Rotation::fromAxisAngle(Vec({0,0,1}), 0), Vec({250, 250}), Vec({320, 240}));
    // auto scene = scene::rectangle3D_002();
    // auto scene = scene::gltf3DBox();
    auto scene = scene::gltf3DSphere();
    // auto scene = scene::gltf3DBoomBox();
    // auto scene = scene::gltf3DDuck();
    // auto scene = scene::simple3DPrism();

    auto render = RenderSample(3);
    render.setOutputFilename("exercise_5");
    render.setCamera(cam)
        .setMode(RenderSample::eMULTITHREADING)
        // .setMode(RenderSample::eSINGLE_RAY)
        // .setMode(RenderSample::eNAIVE)
        .setRecursionDepth(5)
        .enableRayStack(true)
        .setSampleNum(1)
        .setScene(&scene)
        .setTestRay(cam.pixelRay({320, 240}))
        // .setTestRay(Ray(Vec({-0.032410,0.209743,2.759068}), Vec({0.997053,0.075776,-0.011965})))
        .run();

    return 0;
}
