#ifndef COLOR_TRANSFORM_INTERFACE_H_
#define COLOR_TRANSFORM_INTERFACE_H_
#include <vector>

class interface
{
public:
    virtual std::vector<size_t> GetIndex() = 0;
    virtual std::vector<size_t> GetFixedIndex() = 0;
    virtual std::vector<std::pair<size_t, size_t>> GetEdge() = 0;
    virtual std::vector<
        std::tuple<double, double, double, double, double, double>>
    GetPixelPair(size_t lhs_index, size_t rhs_index) = 0;
};
#endif