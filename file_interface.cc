#include "file_interface.hpp"
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

FileInterface::FileInterface() {
  std::string root = "../data_set/";
  for (size_t i = 0; i <= 5; i++) {
    _imgs.push_back(cv::imread(root + "origin_" + std::to_string(i) + ".jpg"));
    _masks.push_back(cv::imread(root + "mask_" + std::to_string(i) + ".jpg",
                                cv::IMREAD_GRAYSCALE));
  }
}
std::vector<size_t> FileInterface::GetIndex() {
  std::vector<size_t> res;
  for (size_t i = 0; i <= 5; i++) {
    res.push_back(i);
  }
  return res;
}

std::vector<size_t> FileInterface::GetFixedIndex() {
  std::vector<size_t> res;
  res.push_back(0);
  return res;
}

std::vector<std::pair<size_t, size_t>> FileInterface::GetEdge() {
  std::vector<std::pair<size_t, size_t>> res;
  res.push_back({0, 1});
  res.push_back({1, 2});
  res.push_back({2, 3});
  res.push_back({3, 4});
  res.push_back({4, 5});
  return res;
}
std::vector<std::tuple<double, double, double, double, double, double>>
FileInterface::GetPixelPair(size_t lhs_index, size_t rhs_index) {
  cv::Mat lhs_img = _imgs[lhs_index];
  cv::Mat rhs_img = _imgs[rhs_index];
  cv::Mat lhs_mask = _masks[lhs_index];
  cv::Mat rhs_mask = _masks[rhs_index];
  // cv::imshow("lhs", lhs_img);
  // cv::imshow("lhs_mask", lhs_mask);
  // cv::waitKey(0);
  std::vector<std::tuple<double, double, double, double, double, double>> res;
  for (size_t row = 0; row < lhs_img.rows; row++) {
    for (size_t col = 0; col < lhs_img.cols; col++) {
      uchar* lhs_ptr = lhs_img.ptr<uchar>(row);
      uchar* rhs_ptr = rhs_img.ptr<uchar>(row);
      if (lhs_mask.at<uchar>(row, col) != 0 &&
          rhs_mask.at<uchar>(row, col) != 0) {
        std::tuple<double, double, double, double, double, double> item;

        std::get<0>(item) = static_cast<double>(lhs_ptr[3 * col + 2]) / 255.0;
        std::get<1>(item) = static_cast<double>(lhs_ptr[3 * col + 1]) / 255.0;
        std::get<2>(item) = static_cast<double>(lhs_ptr[3 * col + 0]) / 255.0;

        std::get<3>(item) = static_cast<double>(rhs_ptr[3 * col + 2]) / 255.0;
        std::get<4>(item) = static_cast<double>(rhs_ptr[3 * col + 1]) / 255.0;
        std::get<5>(item) = static_cast<double>(rhs_ptr[3 * col + 0]) / 255.0;
        printf("lhs_ptr %zu %zu %d %d %d\n", row, col, lhs_ptr[3 * col + 2],
               lhs_ptr[3 * col + 1], lhs_ptr[3 * col]);
        printf("Item : %f %f %f %f %f %f\n", std::get<0>(item),
               std::get<1>(item), std::get<2>(item), std::get<3>(item),
               std::get<4>(item), std::get<5>(item));
        res.push_back(item);
      }
    }
  }
  return res;
}

void FileInterface::ApplyTransform(
    size_t index, std::function<double(double)> r_transform_functor,
    std::function<double(double)> g_transform_functor,
    std::function<double(double)> b_transform_functor) {
  if (index < 0 || index > 5) {
    std::cerr << "Input index error, please check it out" << std::endl;
    return;
  }

  cv::Mat img = _imgs[index];
  cv::Mat mask = _masks[index];
  auto sature_functor = [](double v) {
    if (v < 0) v = 0;
    if (v > 1.0) v = 1.0;

    return static_cast<uchar>(v * 255.0);
  };
  for (size_t row = 0; row < img.rows; row++) {
    for (size_t col = 0; col < img.cols; col++) {
      if (mask.at<uchar>(row, col) > 0) {
        uchar r = img.at<uchar>(row, col * 3 + 2);
        uchar g = img.at<uchar>(row, col * 3 + 1);
        uchar b = img.at<uchar>(row, col * 3 + 0);

        img.at<uchar>(row, col * 3 + 2) =
            sature_functor(r_transform_functor(static_cast<double>(r) / 255.0));
        img.at<uchar>(row, col * 3 + 1) =
            sature_functor(g_transform_functor(static_cast<double>(g) / 255.0));
        img.at<uchar>(row, col * 3 + 0) =
            sature_functor(b_transform_functor(static_cast<double>(b) / 255.0));
      }
    }
  }
}

void FileInterface::MergeImage() {
  cv::Mat ori = _imgs[0].clone();
  for (size_t i = 1; i <= 5; i++) {
    cv::Mat mask = _masks[i];
    cv::Mat img = _imgs[i];

    for (size_t row = 0; row < img.rows; row++) {
      for (size_t col = 0; col < img.cols; col++) {
        if (mask.at<uchar>(row, col) > 0) {
          ori.at<cv::Vec3b>(row, col) = img.at<cv::Vec3b>(row, col);
        }
      }
    }
  }

  cv::imwrite("merge.jpg", ori);
}