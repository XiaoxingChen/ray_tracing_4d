#include "vec3.h"
#include <iostream>
using namespace rtc;
int main(int argc, char const *argv[])
{
    Vector3 p1{1,2,3};
    Vector3 p2{1,2,3};
    Vector3 p3;
    
    std::cout << "p1: " << p1 << std::endl;
    std::cout << "p2: " << p2 << std::endl;
    std::cout << "dot: " << p1.dot(p2) << std::endl;
    std::cout << "+: " << (p1 + p2) << std::endl;
    std::cout << "-: " << p1 - p2 << std::endl;
    std::cout << "*3: " << p1 * 3 << std::endl;
    std::cout << "-(): " << -p1 << std::endl;
    std::cout << "normalized 0: " << p3.normalized()  << std::endl;
    return 0;
}

