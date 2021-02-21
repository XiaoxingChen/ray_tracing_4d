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
    Camera cam(Vec(dim), Rotation::Identity(dim), Vec({500, 500, 50}), Vec({640, 480, 16}));
    // auto scene = scene::simple4D_001();
    // auto scene = scene::gltfTetrahedronInBox(dim);
    // auto scene = scene::gltf4DBox();
    auto scene = scene::gltf4DBoxPrism();

    auto render = RenderSample(dim);
    render.setOutputFilename("exercise_5_4d");
    render.setCamera(cam)
        .setMode(RenderSample::eMULTITHREADING)
        // .setMode(RenderSample::eSINGLE_RAY)
        // .setMode(RenderSample::eNAIVE)
        .setRecursionDepth(5)
        .enableRayStack(true)
        .setSampleNum(1)
        .setScene(&scene)
        // .setTestRay(cam.pixelRay({493, 304, 1}))
        .setTestRay(Ray(Vec::zeros(dim), Vec({0.055599,-0.105241,-0.009928,0.992842})))
        .run();

    return 0;
}
