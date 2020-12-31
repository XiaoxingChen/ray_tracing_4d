#include <iostream>
#include "test_aabb.h"
#include "test_rigid_body.h"
#include "test_linalg.h"
#include "test_ray.h"
#include "test_random_factory.h"




int main(int argc, char const *argv[])
{
  testAABB();
  testRay();
  testLinearAlgebra();
  testRigidBody();
  testRandomFactory();
  std::cout << "done" << std::endl;
  return 0;
}
