#ifndef _FILE_INTERFACE_H_
#define _FILE_INTERFACE_H_
#include <opencv2/core.hpp>
#include <vector>
#include "interface.hpp"

class FileInterface : public interface {
 public:
  FileInterface();
  ~FileInterface() = default;
  virtual std::vector<size_t> GetIndex() override;
  virtual std::vector<size_t> GetFixedIndex() override;
  virtual std::vector<std::pair<size_t, size_t>> GetEdge() override;
  virtual std::vector<
      std::tuple<double, double, double, double, double, double>>
  GetPixelPair(size_t lhs_index, size_t rhs_index) override;

  virtual void ApplyTransform(
      size_t index, std::function<double(double)> r_transform_functor,
      std::function<double(double)> g_transform_functor,
      std::function<double(double)> b_transform_functor) override;
  void MergeImage();

 private:
  std::vector<cv::Mat> _imgs;
  std::vector<cv::Mat> _masks;
};
#endif