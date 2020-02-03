#include <cstring>
#include <functional>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <random>

void AddRandom(cv::Mat &sour, cv::Mat &mask) {
  size_t rows = sour.rows;
  size_t cols = sour.cols;
  std::default_random_engine engine;
  std::uniform_int_distribution<int> dis(0, 15);
  auto rand = std::bind(dis, engine);
  size_t offset = rand();
  for (size_t row = 0; row < rows; row++) {
    for (size_t col = 0; col < cols; col++) {
      if (mask.at<uchar>(row, col) > 0) {
        uchar *p = sour.ptr<uchar>(row);
        // p[col * 3] = (p[col * 3] + offset) % 255;
        // p[col * 3 + 1] = (p[col * 3 + 1] + offset) % 255;
        // p[col * 3 + 2] = (p[col * 3 + 2] + offset) % 255;
      }
    }
  }
}
void MaskImage(cv::Mat &sour, cv::Mat &mask) {
  size_t rows = sour.rows;
  size_t cols = sour.cols;
  for (size_t row = 0; row < rows; row++) {
    for (size_t col = 0; col < cols; col++) {
      if (mask.at<uchar>(row, col) == 0) {
        uchar *p = sour.ptr<uchar>(row);
        p[col * 3] = p[col * 3 + 1] = p[col * 3 + 2] = 0;
      }
    }
  }
}
std::vector<cv::Mat> CreateMask(size_t width, size_t height, size_t num) {
  size_t overlay_window = 50;
  size_t half_overlay_window = 25;
  size_t image_window = width / num;
  std::vector<cv::Mat> res;

  for (size_t i = 0; i < num; i++) {
    cv::Mat mask;
    mask.create(height, width, CV_8UC1);
    size_t start_col = i == 0 ? 0 : (i * image_window - half_overlay_window);
    for (size_t col = 0; col < width; col++) {
      for (size_t row = 0; row < height; row++) {
        if (col >= start_col &&
            col < start_col + image_window + half_overlay_window) {
          mask.at<uchar>(row, col) = 255;
        } else {
          mask.at<uchar>(row, col) = 0;
        }
      }
    }
    res.push_back(mask);
  }
  return res;
}
int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " PATH/TO/image"
              << " SplitNumber" << std::endl;
    return 1;
  }
  cv::Mat img = cv::imread(argv[1]);
  if (img.data == nullptr) {
    std::cerr << "Can't Open " << argv[1] << std::endl;
    return 1;
  }

  size_t rows = img.rows;
  size_t cols = img.cols;
  int split_num = std::atoi(argv[2]);
  if (split_num <= 0) {
    std::cerr << "Split Number less then Zero " << std::endl;
    return 1;
  }

  auto masks = CreateMask(cols, rows, split_num);

  for (size_t i = 0; i < split_num; i++) {
    cv::Mat clone_img = img.clone();
    cv::Mat mask = masks[i];
    MaskImage(clone_img, mask);
    if (i != 0) {
      AddRandom(clone_img, mask);
    }
    cv::imwrite("origin_" + std::to_string(i) + ".jpg", clone_img);
    cv::imwrite("mask_" + std::to_string(i) + ".jpg", mask);
  }
  std::cout << "Hello world" << std::endl;
  return 0;
}