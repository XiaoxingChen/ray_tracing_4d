#ifndef __BASE_TYPE_H__
#define __BASE_TYPE_H__

#include <vector>

namespace rtc{
using float_t = float;
using FloatType = float_t;
// FloatType eps = 1e-7;
inline FloatType eps() {return 1e-7;}
using size_t = std::vector<FloatType>::size_type;

inline FloatType tMin() {return 0.;}
inline FloatType tMax() {return 10000.;}

}//

#endif