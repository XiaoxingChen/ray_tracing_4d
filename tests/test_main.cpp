#include <iostream>
#include "test_aabb.h"
#include "test_linalg.h"



int main(int argc, char const *argv[])
{
  testAABB();
  testLinearAlgebra();
  std::cout << "done" << std::endl;
  return 0;
}
