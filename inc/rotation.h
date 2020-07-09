#if !defined(_ROTATION_H_)
#define _ROTATION_H_
#include "vec3.h"
#include <math.h>

namespace rtc
{
class Mat3: public std::array<V3, 3>
{
    using BaseType = std::array<V3, 3>;
    using ThisType = Mat3;
    public:
        Mat3(): BaseType(){}
        Mat3(const Mat3& rhs): BaseType(rhs){}
        Mat3(std::initializer_list<V3> il)
        {
            if(il.size() != 3)
                throw std::runtime_error(std::string("Size Mismatch!\n") + __FILE__);
            std::copy(il.begin(), il.end(), this->begin());
        }

        static ThisType Identity() { return ThisType{{1,0,0}, {0,1,0}, {0,0,1}}; }

        float_t operator () (int i, int j) const { return at(i).at(j); }
        float_t & operator () (int i, int j) { return at(i).at(j); }

        ThisType operator*= (float_t val) { for(auto & row: *this) row *= val; return *this; }
        ThisType operator* (float_t val) const { return ThisType(*this) *= val; }
        ThisType operator+= (float_t val) { for(auto & row: *this) row += val; return *this; }
        ThisType operator+ (float_t val) const { return ThisType(*this) += val; }

        ThisType operator+= (const ThisType& rhs) { for(int i = 0; i < size(); i++) at(i) += rhs.at(i); return *this; }
        ThisType operator+ (const ThisType& rhs) const { return ThisType(*this) += rhs; }

        V3 dot(const V3& vec) const
        {
            V3 result{0,0,0};
            for(auto & row : *this) result += row.dot(vec);
            return result;
        }

        Mat3 dot(const Mat3& rhs) const
        {
            Mat3 result;
            int size = this->size();
            for(int i = 0; i < size; i++)
            {
                for(int j = 0; j < size; j++)
                {
                    result(i,j) = 0;
                    for(int k = 0; k < size; k++)
                    {
                        result(i, j) += (*this)(i, k) * rhs(k, j);
                    }
                }
            }
        }

};
class Rotation
{
public:
    Rotation(/* args */) {}
    ~Rotation() {}
    using ThisType = Rotation;
    ThisType operator*(const ThisType& rhs) const;

    V3 apply(const V3& vector);
    
    template <typename T>
    T apply(const T& vec_container) const
    {
        T result(vec_container.begin(), vec_container.end());
        Mat3 R = rodrigues();
        for(int i = 0; i < result.size(); i++)
        {
            result.at(i) = R.dot(vec_container.at(i));
        }
        return result;
    }

private:

    Rotation(const Mat3& R);
    Mat3 rodrigues() const;


    UnitVector3 axis_;
    float_t angle_;
};
} // namespace rtc



#endif // _ROTATION_H_
