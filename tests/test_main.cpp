#include <iostream>
#include "test_aabb.h"
#include "test_rigid_body.h"
#include "test_linalg.h"
#include "test_ray.h"
#include "test_random_factory.h"
#include "test_material.h"
#include "test_rotation.h"
#include "test_bvh.h"






int main(int argc, char const *argv[])
{
  testAABB();
  testRay();
  testLinearAlgebra();
  testRotation();
  testRigidBody();
  testPrimitiveGeometry();
  testRandomFactory();
  testMaterial();
  testBvh();
  std::cout << "done" << std::endl;
  return 0;
}
