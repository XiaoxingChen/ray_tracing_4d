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
    // auto scene = scene::rectangle3D_002();
    auto scene = scene::simple3D_003();

    auto render = RenderSample(3);
    render.setOutputFilename("exercise_5.ppm");
    render.setCamera(cam)
        .setMode(RenderSample::eMULTITHREADING)
        // .setMode(RenderSample::eSINGLE_RAY)
        // .setMode(RenderSample::eNAIVE)
        .setRecursionDepth(15)
        .enableRayStack(true)
        .setSampleNum(10)
        .setScene(&scene)
        .setTestRay(cam.pixelRay({493, 304}))
        // .setTestRay(Ray(Vec({0.8857144713,1.0000002384,14.2857179642}), Vec({0.0617306866,-0.0696959719,0.9956564903})))
        .run();

    return 0;
}
