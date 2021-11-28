#ifndef __IO_H__
#define __IO_H__
#include <vector>
#include <string>
// #include "vec3.h"
#include <array>
#include "mxm/common.h"
#include <fstream>
#include "linalg.h"
#include <iostream>
#include "mxm/cv_pixel.h"
using namespace mxm;
namespace rtc{

using PixelCoordinates = std::vector<std::vector<size_t>>;

template<typename iterator>
inline void writeToPPM(const std::string& filename, int w, int h, iterator begin, iterator end)
{
    auto f = std::fstream(filename, std::ios::out );
    if(end - begin != w * h)
    {
        std::cout << "size mismatch! w:" << w <<", h: " << h
        << ", w*h: " << w*h << ", real size: " << end - begin << std::endl;
    }
    f << "P3\n" << w << " " << h << "\n255\n";
    // for(const auto & px: pixs)
    for(auto it = begin; it != end; it++)
        f << it->rU8() << " " << it->gU8() <<  " " << it->bU8() << "\n";

    f.close();
}

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

    for(size_t j = 0; j < height; j++)
    {
        for(size_t i = 0; i < width; i++)
        {
            ret.push_back(std::vector<size_t>{i, j});
        }
    }
    return ret;
}

inline PixelCoordinates PPMCoordinateSequence(size_t width, size_t height, size_t thickness)
{
    PixelCoordinates ret;

    for(size_t k = 0; k < thickness; k++)
    {
        for(size_t j = 0; j < height; j++)
        {
            for(size_t i = 0; i < width; i++)
            {
                ret.push_back(std::vector<size_t>{i, j, k});
            }
        }
    }
    return ret;
}


}// rtc
#endif