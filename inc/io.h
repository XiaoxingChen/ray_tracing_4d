#ifndef __IO_H__
#define __IO_H__
#include <vector>
#include <string>
#include "vec3.h"
#include <fstream>

namespace rtc{

using PixelCoordinates = std::vector<std::array<int, 2>>;

template<template<typename ...> class Container>
inline void writeToPPM(const std::string& filename, int w, int h, const Container<Pixel>& pixs)
{
    auto f = std::fstream(filename, std::ios::out );
    if(pixs.size() != w * h)
    {
        std::cout << "size mismatch! w:" << w <<", h: " << h 
        << ", w*h: " << w*h << ", pixs.size(): " << pixs.size() << std::endl;
    }
    f << "P3\n" << w << " " << h << "\n255\n";
    for(const auto & px: pixs) 
        f << px.rU8() << " " << px.gU8() <<  " " << px.bU8() << "\n";
    
    f.close();
}

inline PixelCoordinates PPMCoordinateSequence(int width, int height)
{
    PixelCoordinates ret;
    for(int j = height - 1; j >= 0; j--)
    {
        for(int i = 0; i < width; i++)    
        {
            ret.push_back(std::array<int, 2>{i, j});
        }
    }
    return ret;
}

}// rtc
#endif