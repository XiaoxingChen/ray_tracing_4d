#if !defined(_PIXEL_H_)
#define _PIXEL_H_

#include <array>
#include <cmath>
#include <string>
#include "base_type.h"
#include "accessor.h"
#include "stb_image.h"
#include "stb_image_write.h"


namespace rtc
{

template<typename DType>
inline int quantizeToU8(DType val)
{
    return val < 0 ? 0 : val > 1 ? 255 : val * 255.99;
}

template<>
inline int quantizeToU8<uint8_t>(uint8_t val) { return val; }

template<typename DeriveType, typename DType>
// class RGBA_F32
class RGBA
{
public:
    // using DType = float;
    // using DType = typename DeriveType::dtype;
    DeriveType* deriveThis() {return reinterpret_cast<DeriveType*>(this);}
    const DeriveType* deriveThis() const {return reinterpret_cast<const DeriveType*>(this);}
    const DType& at(size_t i) const {return (*deriveThis())(i);}

    const DType & r() const {return (*deriveThis())(0);}
    const DType & g() const {return (*deriveThis())(1);}
    const DType & b() const {return (*deriveThis())(2);}
    const DType & a() const {return deriveThis()->size() > 2 ? (*deriveThis())(3) : 0.;}

    int rU8() const { return quantizeToU8<DType>(r()); }
    int gU8() const { return quantizeToU8<DType>(g()); }
    int bU8() const { return quantizeToU8<DType>(b()); }
    int aU8() const { return quantizeToU8<DType>(a()); }
};

template<typename DType, size_t NChannel>
class PixelType: public LinearData<PixelType<DType, NChannel>, DType>, public RGBA<PixelType<DType,NChannel>, DType>
{
public:
    // static const size_t NChannel = 3;
    using InitialType = std::array<DType, NChannel>;
    using ThisType = PixelType<DType,NChannel>;
    // using DType = float;
    using BaseType0 = LinearData<PixelType<DType,NChannel>, DType>;
    PixelType(const InitialType& v): data_(v){}
    PixelType(){ BaseType0::traverse([&](size_t i){(*this)(i) = 0;}); }

    const DType& operator () (size_t i) const { return data_.at(i); }
    DType& operator () (size_t i) { return data_.at(i); }

    static ThisType black() {return ThisType();}
    static ThisType white() {return ThisType() + 1;}
    static constexpr size_t size() {return NChannel;}

    enum{
        N_CHANNEL = NChannel
    };

    // constexpr DType dtype() const;
    // using dtype = DType;

private:
    std::array<DType, NChannel> data_;
};

using Pixel = PixelType<float, 3>;
using Pixel4 = PixelType<float, 4>;
using PixelR8G8B8 = PixelType<uint8_t, 3>;
using PixelR8G8B8A8 = PixelType<uint8_t, 4>;

// template<typename DType, size_t NChannel>
// inline Matrix<PixelType<uint8_t, NChannel>>
// imread(const std::string& filename)
// {
//     throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));
//     return Matrix<PixelType<uint8_t, NChannel>>();
// }

template<typename DType, size_t NChannel>
inline typename
std::enable_if<std::is_same<DType,uint8_t>::value,
Matrix<PixelType<DType,NChannel>> >::type
imread(const std::string& filename)
{
    int width, height, n;
    uint8_t* data = stbi_load(filename.c_str(), &width, &height, &n, NChannel);
    if(!data)
        throw std::runtime_error(std::string(__FILE__) + ":" + std::to_string(__LINE__));

    Matrix<PixelType<DType, NChannel>> ret({ (size_t)height, (size_t)width});
    const size_t& ch(NChannel);
    ret.traverse([&](auto i, auto j)
    {
        for(size_t k = 0; k < ch; k++)
        {
            ret(i,j)(k) = data[i * width * ch + j * ch + k];
        }
    });
    stbi_image_free(data);
    return ret;
}

template<typename DType, size_t NChannel>
inline typename
std::enable_if<std::is_same<DType,float>::value,
Matrix<PixelType<DType,NChannel>> >::type
imread(const std::string& filename)
{
    using ReturnType = Matrix<PixelType<DType, NChannel>>;
    Matrix<PixelType<uint8_t, NChannel>> image_u8 = imread<uint8_t, NChannel>(filename);
    ReturnType image_f32(image_u8.shape());
    const size_t& ch(NChannel);
    image_f32.traverse([&](auto i, auto j){
        for(size_t k = 0; k < ch; k++)
            image_f32(i,j)(k) = image_u8(i,j)(k) * (1./ 255.);
    });
    return image_f32;
}

inline Matrix<Pixel> loadImage(const std::string& filename)
{
    return imread<float, 3>(filename);
}

inline std::string getFileExt(const std::string& s)
{
   size_t i = s.rfind('.', s.length());
   if (i != std::string::npos) {
      return(s.substr(i+1, s.length() - i));
   }

   return("");
}

template<typename DType, size_t NChannel>
inline typename std::enable_if<std::is_same<DType, uint8_t>::value, std::vector<uint8_t>>::type
serialize(const Matrix<PixelType<DType, NChannel>>& img)
{
    std::vector<uint8_t> src_mem(NChannel * img.shape(0) * img.shape(1));
    img.traverse([&](auto i, auto j){
        for(size_t k = 0; k < NChannel; k++)
        {
            src_mem.at(i * img.shape(1) * NChannel + j * NChannel + k) = img(i,j)(k);
        }
    });
    return src_mem;
}

template<typename DType, size_t NChannel>
inline typename std::enable_if<std::is_same<DType, float>::value, std::vector<uint8_t>>::type
serialize(const Matrix<PixelType<DType, NChannel>>& img)
{
    std::vector<uint8_t> src_mem(NChannel * img.shape(0) * img.shape(1));
    img.traverse([&](auto i, auto j){
        for(size_t k = 0; k < NChannel; k++)
        {
            src_mem.at(i * img.shape(1) * NChannel + j * NChannel + k) = quantizeToU8(img(i,j)(k));
        }
    });
    return src_mem;
}


template<typename DType, size_t NChannel>
void imwrite(const std::string& filename, const Matrix<PixelType<DType, NChannel>>& img)
{
    size_t height = img.shape(0);
    size_t width = img.shape(1);
    std::string ext = getFileExt(filename);

    auto src_mem = serialize(img);

    if(ext == std::string("png"))
    {
        stbi_write_png(filename.c_str(), img.shape(1), img.shape(0), NChannel, src_mem.data(), img.shape(1) * NChannel);
    }else if(ext == std::string("bmp"))
    {
        stbi_write_bmp(filename.c_str(), img.shape(1), img.shape(0), NChannel, src_mem.data());
    }else if(ext == std::string("jpg"))
    {
        stbi_write_jpg(filename.c_str(), img.shape(1), img.shape(0), NChannel, src_mem.data(), 100);
    }else
    {
        std::cout << "format " << ext << " not supported" << std::endl;
    }

}

} // namespace rtc

#endif // _PIXEL_H_
