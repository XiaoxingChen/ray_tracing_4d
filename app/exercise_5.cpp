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
    Camera cam(Vec(3), Rotation::fromAxisAngle(Vec({0,0,1}), 0), Vec({500, 500}), Vec({320, 240}));
    auto scene = scene::simple3D_003();
    auto render = RenderSample(3);
    render.setTargetPixel({330, 260});
    render.setOutputFilename("exercise_5.ppm");
    render.setCamera(cam)
        .setMode(RenderSample::eMULTITHREADING)
        .setRecursionDepth(10)
        .enableRayStack(true)
        .setSampleNum(10)
        .setScene(&scene)
        .run();

    return 0;
}
