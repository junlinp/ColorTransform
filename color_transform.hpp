#ifndef COLOR_TRANSFORM_COLOR_TRANSFORM_H_
#define COLOR_TRANSFORM_COLOR_TRANSFORM_H_
#include "Eigen/Dense"
#include "interface.hpp"
#include "src/QuadProg++.hh"

class ColorTransform {
 public:
  ColorTransform() = default;
  ~ColorTransform() = default;
  void setInterface(interface *_interface);
  void process();

 private:
  interface *_interface;
  ColorTransform(const ColorTransform &);
  ColorTransform &operator=(const ColorTransform &);

  void GenerateQuadriticFormula();
  void GenerateSoftLimitedFormula();
  void GenerateFixedFormula();
  // void MergeequalFormula();
  void Solve();

  bool MinimumQuadraticProgram();
  quadprogpp::Matrix<double> EigenMatrixToQuadProgMatrix(
      const Eigen::Ref<const Eigen::MatrixXd> &matrix);

  quadprogpp::Vector<double> EigenVectorToQuadProgVector(
      const Eigen::Ref<const Eigen::VectorXd> &vector);

  Eigen::VectorXd f_coefficient(double v);

  size_t _node_size;
  size_t _fixed_image_size;
  Eigen::MatrixXd _r_quadratic_matrix;
  Eigen::MatrixXd _g_quadratic_matrix;
  Eigen::MatrixXd _b_quadratic_matrix;
  Eigen::VectorXd _r_linear_vector;
  Eigen::VectorXd _g_linear_vector;
  Eigen::VectorXd _b_linear_vector;

  Eigen::MatrixXd _range_quadratic_matrix;
  Eigen::VectorXd _range_linear_vector;

  Eigen::MatrixXd _equal_formula_matrix;
  Eigen::VectorXd _equal_formula_constant;
};
#endif