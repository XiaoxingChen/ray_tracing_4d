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
    auto scene = scene::rectangle3D_002();
    // auto scene = scene::simple3D_003();

    auto render = RenderSample(3);
    render.setOutputFilename("exercise_5.ppm");
    render.setCamera(cam)
        .setMode(RenderSample::eMULTITHREADING)
        // .setMode(RenderSample::eSINGLE_RAY)
        // .setMode(RenderSample::eNAIVE)
        .setRecursionDepth(13)
        .enableRayStack(true)
        .setSampleNum(5)
        .setScene(&scene)
        .setTestRay(cam.pixelRay({162,161}))
        // .setTestRay(Ray(Vec({-2.149441,-0.474552,6.978703}), Vec({0.223787,0.254858,0.940727})))
        .run();

    return 0;
}
