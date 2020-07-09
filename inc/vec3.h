#ifndef __VEC3_H__
#define __VEC3_H__
#include <array>
#include <math.h>
#include <iostream>
#include "base_type.h"
#include <initializer_list>


namespace rtc{

#define VECTOR_SCALAR_OPERATOR(op, op_eq) \
ThisType& operator op_eq (float_t val) { for(int i = 0; i < Dim; i++) v_.at(i) op_eq val; return *this;} \
ThisType operator op (float_t val) const { return (ThisType(*this) op_eq val);}  

#define VECTOR_VECTOR_OPERATOR(op, op_eq) \
ThisType& operator op_eq (const ThisType& val) { for(int i = 0; i < Dim; i++) v_.at(i) op_eq val.at(i); return *this;} \
ThisType operator op (const ThisType& val) const { return (ThisType(*this) op_eq val);}  

#define SCALAR_VECTOR_OPERATOR(ThisType) \
inline ThisType operator + (float_t lhs, const ThisType& rhs) { return rhs + lhs;}  \
inline ThisType operator - (float_t lhs, const ThisType& rhs) { return -rhs + lhs;} \
inline ThisType operator * (float_t lhs, const ThisType& rhs) { return rhs * lhs;} 


template<uint Dim>
class Vector
{
public: 
    using ThisType = Vector<Dim>;
    using ThisTypeIn = const Vector<Dim>&;
    Vector(){ for(auto & val: v_) val = 0;}
    Vector(const ThisType& in):v_(in.v_){}
    Vector(std::initializer_list<float_t> il)
    {
        if(il.size() != Dim)
            throw std::runtime_error(std::string("Size Mismatch!\n") + __FILE__);
        std::copy(il.begin(), il.end(), v_.begin());
    }
    
    const float_t& at(int i) const {return v_.at(i);}
    float_t& at(int i) {return v_.at(i);}

    VECTOR_SCALAR_OPERATOR(+, +=);
    VECTOR_SCALAR_OPERATOR(-, -=);
    VECTOR_SCALAR_OPERATOR(*, *=);
    VECTOR_SCALAR_OPERATOR(/, /=);

    VECTOR_VECTOR_OPERATOR(+, +=);
    VECTOR_VECTOR_OPERATOR(-, -=);
    VECTOR_VECTOR_OPERATOR(*, *=);
    VECTOR_VECTOR_OPERATOR(/, /=);

    ThisType operator -() const        { return ThisType(*this) *= -1;}


    float_t dot(ThisTypeIn rhs) const 
    { 
        float_t sum(0); 
        for(int i = 0; i < Dim; i++) sum += (this->at(i) * rhs.at(i));
        return sum;
    }

    float_t norm() const { return sqrt(this->dot(*this)); }
    ThisType normalized() const { ThisType tmp(*this); tmp.normalize(); return tmp; }
    void normalize() { if(this->norm() > 1e-5) *this /= this->norm(); else this->v_.at(0) = 1; }
    

protected:
    std::array<float_t, Dim> v_;
};

#define VECTOR_SCALAR_OPERATOR_DERIV(op, op_eq) \
ThisType& operator op_eq (float_t val) { BaseType::operator op_eq (val); return *this;} \
ThisType operator op (float_t val) const { return BaseType::operator op (val); }

#define VECTOR_VECTOR_OPERATOR_DERIV(op, op_eq) \
ThisType& operator op_eq (const ThisType& val) { BaseType::operator op_eq (val); return *this;} \
ThisType operator op (const ThisType& val) const { return BaseType::operator op (val); }


#define VECTOR_DERIVE(this_type, base_type) \
    using BaseType = base_type; \
    using ThisType = this_type; \
    this_type(std::initializer_list<float_t> il): BaseType(il){} \
    this_type(const BaseType& in):BaseType(in){} \
    this_type():BaseType(){} \
    VECTOR_SCALAR_OPERATOR_DERIV(+, +=) \
    VECTOR_SCALAR_OPERATOR_DERIV(-, -=) \
    VECTOR_SCALAR_OPERATOR_DERIV(*, *=) \
    VECTOR_SCALAR_OPERATOR_DERIV(/, /=) \
    VECTOR_VECTOR_OPERATOR_DERIV(+, +=) \
    VECTOR_VECTOR_OPERATOR_DERIV(-, -=) \
    VECTOR_VECTOR_OPERATOR_DERIV(*, *=) \
    ThisType operator -() const { return BaseType::operator -(); } \
    ThisType normalized() const { return (ThisType(BaseType::normalized())); }

class Vector3: public Vector<3>
{
    public:
        VECTOR_DERIVE(Vector3, Vector);

        const float_t & x() const {return this->v_.at(0);}
        const float_t & y() const {return this->v_.at(1);}
        const float_t & z() const {return this->v_.at(2);}

        Vector3 cross(const Vector3& rhs)
        {
            return Vector3{
                v_.at(1) * rhs.at(2) - v_.at(2) * rhs.at(1), 
                -v_.at(0) * rhs.at(2) + v_.at(2) * rhs.at(0),
                v_.at(0) * rhs.at(1) - v_.at(1) * rhs.at(0)};
        }

        static Vector3 ones() { return Vector3{1,1,1}; }

    private:
};

SCALAR_VECTOR_OPERATOR(Vector3);

using V3in = const Vector3&;
using V3 = Vector3;

class UnitVector3 :public Vector3
{
public:
    using ThisType = UnitVector3;
    using BaseType = Vector3;
    
    UnitVector3(std::initializer_list<float_t> il): BaseType(il) { this->normalize(); } 
    UnitVector3(const BaseType& in):BaseType(in) { this->normalize(); } 
    UnitVector3():BaseType(V3{1,0,0}) {} 
    
    
private:
    using Vector3::normalized;
    using Vector::normalized;
    /* data */
};

class Pixel: public Vector<3>
{
    public:
        VECTOR_DERIVE(Pixel, Vector);

        const float_t & r() const {return this->v_.at(0);}
        const float_t & g() const {return this->v_.at(1);}
        const float_t & b() const {return this->v_.at(2);}

        int rU8() const {return r() < 0 ? 0 : r() > 1 ? 255 : r() * 255.99;}
        int gU8() const {return g() < 0 ? 0 : g() > 1 ? 255 : g() * 255.99;}
        int bU8() const {return b() < 0 ? 0 : b() > 1 ? 255 : b() * 255.99;}

        const float_t & x() const = delete;
        const float_t & y() const = delete;
        const float_t & z() const = delete;

};

SCALAR_VECTOR_OPERATOR(Pixel);

}//rtc

template <typename T>
inline std::ostream& cout3d(std::ostream &os, const T& rhs)
{
    os << "[";
    for(int i = 0; i < 3; i++) os << " " << rhs.at(i);
    os << "]";
    return os;
}

inline std::ostream& operator<<(std::ostream &os, const rtc::Vector3& rhs)
{
    return cout3d(os, rhs);
}

inline std::ostream& operator<<(std::ostream &os, const rtc::Pixel& rhs)
{
    return cout3d(os, rhs);
}
#endif
