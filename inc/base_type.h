#ifndef __BASE_TYPE_H__
#define __BASE_TYPE_H__

#include <vector>

namespace rtc{
using float_t = float;
using FloatType = float_t;
// FloatType eps = 1e-7;
inline constexpr FloatType eps() {return 1e-7;}
using size_t = std::vector<FloatType>::size_type;

inline constexpr FloatType tMin() {return 1e-5;}
inline constexpr FloatType tMax() {return 10000.;}

}//

#endif