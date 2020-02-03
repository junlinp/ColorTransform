#include "color_transform.hpp"
#include <Eigen/Eigenvalues>
#include <iostream>
#include <map>

void ColorTransform::setInterface(interface *_interface) {
  this->_interface = _interface;
}
void ColorTransform::process() {
  if (_interface == nullptr) {
    std::cerr << "Have not set the interface " << std::endl;
    return;
  }
  _node_size = _interface->GetIndex().size();
  _fixed_image_size = _interface->GetFixedIndex().size();

  GenerateQuadriticFormula();
  GenerateSoftLimitedFormula();
  GenerateFixedFormula();
  // MergeEqualFormula();
  Solve();
  // post process
}
void ColorTransform::GenerateQuadriticFormula() {
  std::map<size_t, size_t> id_map_idx;
  size_t counter = 0;
  for (size_t id : _interface->GetIndex()) {
    id_map_idx[id] = counter++;
  }
  auto graph = _interface->GetEdge();
  _r_quadratic_matrix = Eigen::MatrixXd::Zero(15 * _node_size, 15 * _node_size);
  _g_quadratic_matrix = Eigen::MatrixXd::Zero(15 * _node_size, 15 * _node_size);
  _b_quadratic_matrix = Eigen::MatrixXd::Zero(15 * _node_size, 15 * _node_size);
  for (auto edge : graph) {
    std::cout << "Process Edge " << edge.first << " " << edge.second
              << std::endl;
    auto pixelpairs = _interface->GetPixelPair(edge.first, edge.second);
    std::cout << "Pixel Pairs Count : " << pixelpairs.size() << std::endl;
    size_t lhs_index = id_map_idx[edge.first];
    size_t rhs_index = id_map_idx[edge.second];

    Eigen::MatrixXd r_sub_matrix =
        Eigen::MatrixXd::Zero(pixelpairs.size(), 15 * _node_size);
    Eigen::MatrixXd g_sub_matrix(pixelpairs.size(), 15 * _node_size);
    g_sub_matrix.setZero();
    Eigen::MatrixXd b_sub_matrix(pixelpairs.size(), 15 * _node_size);
    b_sub_matrix.setZero();
    size_t index = 0;
    for (auto pixelpair : pixelpairs) {
      Eigen::VectorXd r_lhs_vec = f_coefficient(std::get<0>(pixelpair));
      Eigen::VectorXd g_lhs_vec = f_coefficient(std::get<1>(pixelpair));
      Eigen::VectorXd b_lhs_vec = f_coefficient(std::get<2>(pixelpair));

      Eigen::VectorXd r_rhs_vec = f_coefficient(std::get<3>(pixelpair));
      Eigen::VectorXd g_rhs_vec = f_coefficient(std::get<4>(pixelpair));
      Eigen::VectorXd b_rhs_vec = f_coefficient(std::get<5>(pixelpair));
      r_sub_matrix.block<1, 15>(index, lhs_index * 15) = r_lhs_vec.transpose();
      r_sub_matrix.block<1, 15>(index, rhs_index * 15) = -r_rhs_vec.transpose();
      g_sub_matrix.block<1, 15>(index, lhs_index * 15) = g_lhs_vec.transpose();
      g_sub_matrix.block<1, 15>(index, rhs_index * 15) = -g_rhs_vec.transpose();
      b_sub_matrix.block<1, 15>(index, lhs_index * 15) = b_lhs_vec.transpose();
      b_sub_matrix.block<1, 15>(index, rhs_index * 15) = -b_rhs_vec.transpose();
      index++;
    }
    std::cout << "here" << std::endl;
    _r_quadratic_matrix += r_sub_matrix.transpose() * r_sub_matrix;
    _g_quadratic_matrix += g_sub_matrix.transpose() * g_sub_matrix;
    _b_quadratic_matrix += b_sub_matrix.transpose() * b_sub_matrix;
  }
}

void ColorTransform::GenerateSoftLimitedFormula() {
  _range_quadratic_matrix =
      Eigen::MatrixXd::Zero(15 * _node_size, 15 * _node_size);
  _range_linear_vector = Eigen::VectorXd::Zero(15 * _node_size);

  Eigen::MatrixXd p = Eigen::MatrixXd::Zero(256 * _node_size, 15 * _node_size);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(256 * _node_size);
  for (size_t node_index = 0; node_index < _node_size; node_index++) {
    for (size_t i = 0; i < 256; i++) {
      double v = static_cast<double>(i) / 255.0;
      p.block<1, 15>(node_index * 256 + i, 15 * node_index) = f_coefficient(v);
      q(node_index * 256 + i) = v;
    }
  }
  _range_quadratic_matrix = p.transpose() * p;
  _range_linear_vector = q.transpose() * p;
}
void ColorTransform::GenerateFixedFormula() {
  std::vector<size_t> index_vec = _interface->GetIndex();
  std::map<size_t, size_t> map_index_to_idx;
  for (size_t i = 0; i < index_vec.size(); i++) {
    map_index_to_idx[index_vec[i]] = i;
  }

  std::vector<size_t> fixed_index = _interface->GetFixedIndex();
  _equal_formula_matrix =
      Eigen::MatrixXd::Zero(fixed_index.size() * 15, 15 * _node_size);
  _equal_formula_constant = Eigen::VectorXd::Zero(fixed_index.size() * 15);
  int i = 0;
  for (size_t idx : fixed_index) {
    size_t index = map_index_to_idx[idx];
    _equal_formula_matrix.block<15, 15>(i * 15, index * 15) =
        Eigen::MatrixXd::Identity(15, 15);

    for (size_t j = 0; j < 5; j++) {
      _equal_formula_constant(i * 15 + 3 * j + 0) =
          -0.2 * static_cast<double>(j);
      _equal_formula_constant(i * 15 + 3 * j + 1) = -1.0;
      _equal_formula_constant(i * 15 + 3 * j + 2) = 0.0;
    }
    i++;
  }
}

void ColorTransform::Solve() {
  std::cout << _r_quadratic_matrix << std::endl;
  const size_t ROW = _r_quadratic_matrix.rows();
  const size_t COL = _r_quadratic_matrix.cols();
  size_t count = 0;
  for (size_t row = 0; row < ROW; row++) {
    for (size_t col = 0; col < COL; col++) {
      if (abs(_r_quadratic_matrix(row, col)) > 1e-6) {
        count++;
      }
    }
  }
  std::cout << "None Zero Count : " << count << std::endl;
  Eigen::EigenSolver<Eigen::MatrixXd> es(_r_quadratic_matrix +
                                         0.5 * _range_quadratic_matrix);
  std::cout << "The eigenvalues of _r_quadratic_matrix : " << es.eigenvalues()
            << std::endl;

  MinimumQuadraticProgram();
}
bool ColorTransform::MinimumQuadraticProgram() {
  quadprogpp::Matrix<double> Q =
      EigenMatrixToQuadProgMatrix(this->_range_quadratic_matrix);
  quadprogpp::Vector<double> g =
      EigenVectorToQuadProgVector(this->_range_linear_vector);

  quadprogpp::Matrix<double> CE =
      EigenMatrixToQuadProgMatrix(_equal_formula_matrix);
  quadprogpp::Vector<double> ce0 =
      EigenVectorToQuadProgVector(_equal_formula_constant);

  quadprogpp::Matrix<double> CI;
  CI.resize(15 * _node_size, 0);
  quadprogpp::Vector<double> ci0;
  ci0.resize(0);
  quadprogpp::Vector<double> x;
  x.resize(15 * _node_size);
  double error = quadprogpp::solve_quadprog(Q, g, CE, ce0, CI, ci0, x);

  std::cout << "Error : " << error << std::endl;
  std::cout << "X : ";
  for (size_t i = 0; i < 15; i++) {
    std::cout << x[i] << " ";
  }
  std::cout << std::endl;
  return false;
}
quadprogpp::Matrix<double> ColorTransform::EigenMatrixToQuadProgMatrix(
    const Eigen::Ref<const Eigen::MatrixXd> &matrix) {
  size_t ROW = matrix.rows();
  size_t COL = matrix.cols();
  quadprogpp::Matrix<double> res;
  if (ROW == 0) {
    res.resize(COL, 0);
    return res;
  }

  res.resize(COL, ROW);
  for (size_t row = 0; row < COL; row++) {
    for (size_t col = 0; col < ROW; col++) {
      res[row][col] = matrix(col, row);
    }
  }
  return res;
}

quadprogpp::Vector<double> ColorTransform::EigenVectorToQuadProgVector(
    const Eigen::Ref<const Eigen::VectorXd> &vector) {
  size_t ROW = vector.rows();
  quadprogpp::Vector<double> res;
  if (ROW == 0) {
    res.resize(0);
    return res;
  }

  res.resize(ROW);
  for (size_t i = 0; i < ROW; i++) {
    res[i] = vector(i, 0);
  }
  return res;
}

Eigen::VectorXd ColorTransform::f_coefficient(double v) {
  Eigen::VectorXd res = Eigen::VectorXd::Zero(15);
  if (v < 0) {
    std::cerr << "Waring: f_coefficient recvive a value less then Zero"
              << std::endl;
    return res;
  }

  if (v > 1.0) {
    std::cerr << "Waring: f_coefficient recvive a value more then One"
              << std::endl;
  }

  int index = static_cast<int>(v / 0.2);
  if (index == 5) {
    index = 4;
  }
  res(index * 3, 0) = 1.0;
  res(index * 3 + 1, 0) = v - 0.2 * index;
  res(index * 3 + 2, 0) = (v - 0.2 * index) * (v - 0.2 * index);
  return res;
}