#ifndef COLOR_TRANSFORM_INTERFACE_H_
#define COLOR_TRANSFORM_INTERFACE_H_
#include <functional>
#include <vector>
class interface {
 public:
  virtual std::vector<size_t> GetIndex() = 0;
  virtual std::vector<size_t> GetFixedIndex() = 0;
  virtual std::vector<std::pair<size_t, size_t>> GetEdge() = 0;
  virtual std::vector<
      std::tuple<double, double, double, double, double, double>>
  GetPixelPair(size_t lhs_index, size_t rhs_index) = 0;
  virtual void ApplyTransform(
      size_t index, std::function<double(double)> r_transform_functor,
      std::function<double(double)> g_transform_functor,
      std::function<double(double)> b_transform_functor) = 0;
};
#endif