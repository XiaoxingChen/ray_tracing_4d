#ifndef __IO_H__
#define __IO_H__
#include <vector>
#include <string>
// #include "vec3.h"
#include <array>
#include "base_type.h"
#include <fstream>
#include "linalg.h"
#include <iostream>

namespace rtc{

using PixelCoordinates = std::vector<std::array<size_t, 2>>;

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

inline PixelCoordinates PPMCoordinateSequence(size_t width, size_t height)
{
    PixelCoordinates ret;
    // for(int j = height - 1; j >= 0; j--)
    for(size_t j = 0; j < height; j++)
    {
        for(size_t i = 0; i < width; i++)
        {
            ret.push_back(std::array<size_t, 2>{i, height - j - 1});
        }
    }
    return ret;
}

}// rtc
#endif