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
    size_t dim = 4;
    Camera cam(Vec(dim), Rotation::Identity(dim), Vec({500, 500, 15}), Vec({640, 480, 1}));
    auto scene = scene::simple4D_001();
    // auto scene = scene::gltfTetrahedronInBox(dim);
    // auto scene = scene::gltf4DBox();

    auto render = RenderSample(dim);
    render.setOutputFilename("exercise_5_4d");
    render.setCamera(cam)
        .setMode(RenderSample::eMULTITHREADING)
        // .setMode(RenderSample::eSINGLE_RAY)
        // .setMode(RenderSample::eNAIVE)
        .setRecursionDepth(2)
        .enableRayStack(true)
        .setSampleNum(1)
        .setScene(&scene)
        .setTestRay(cam.pixelRay({493, 304, 1}))
        .run();

    return 0;
}
