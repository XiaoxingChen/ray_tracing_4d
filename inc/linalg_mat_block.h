#if !defined(_LINALG_MAT_BLOCK_H_)
#define _LINALG_MAT_BLOCK_H_

#include "linalg_mat.h"

namespace rtc
{

struct Indexer
{
    Indexer(int from_end_)
    :from_end(from_end_){}

    int from_end;
    Indexer operator - (int i)
    { return Indexer(from_end - i); }
};

inline Indexer end()
{
    return Indexer(0);
}

class Block
{
public:
    Block(const std::array<size_t, 2>& row, const std::array<size_t, 2>& col)
        :row01_col01_({row[0], row[1], col[0], col[1]}) {}

    template<typename DType>
    std::array<size_t, 4> getBlock(const Matrix<DType>& mat) const
    {
        std::array<size_t, 4> ret(row01_col01_);
        if(0 == ret[1]) ret[1] = mat.shape(0);
        if(0 == ret[3]) ret[3] = mat.shape(1);
        return ret;
    }

protected:
    std::array<size_t, 4> row01_col01_;
};

// inline Mat& Mat::set(const Block& s, const Mat& rhs)
// {
//     auto row01_col01 = s.getBlock(*this);
//     return Mat::setBlock(row01_col01[0], row01_col01[2], rhs);
// }

inline Block Row(size_t idx)
{
    return Block({idx, idx + 1}, {0,0});
}


inline Block Col(size_t idx)
{
    return Block({0,0}, {idx, idx + 1});
}

} // namespace rtc
#endif // _LINALG_MAT_BLOCK_H_
