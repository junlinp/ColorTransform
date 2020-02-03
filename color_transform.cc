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
  auto index_vector = _interface->GetIndex();
  _node_size = _interface->GetIndex().size();
  _fixed_image_size = _interface->GetFixedIndex().size();
  for (size_t i = 0; i < _node_size; i++) {
    _id_to_idx[index_vector[i]] = i;
  }
  GenerateQuadriticFormula();
  GenerateSoftLimitedFormula();
  GenerateHardLimitedFormula();
  GenerateFixedFormula();
  GenerateSplineFormula();
  // MergeEqualFormula();
  Solve();
  // post process

  auto linear_functor = [](double v) { return v; };
  for (size_t img_id : index_vector) {
    size_t index = _id_to_idx[img_id];
    auto r_functor = [this, &index](double v) {
      if (v < 0) v = 0;
      if (v > 1.0) v = 1.0;
      Eigen::VectorXd coeffcient = this->f_coefficient(v);
      Eigen::VectorXd param = this->_r_params.block<15, 1>(index * 15, 0);
      double value = param.dot(coeffcient);
      std::cout << index << " R : " << v << " map to " << value << std::endl;
      return value;
    };
    auto g_functor = [this, &index](double v) {
      if (v < 0) v = 0;
      if (v > 1.0) v = 1.0;
      Eigen::VectorXd coeffcient = this->f_coefficient(v);
      Eigen::VectorXd param = this->_g_params.block<15, 1>(index * 15, 0);
      double value = param.dot(coeffcient);
      std::cout << index << " G : " << v << " map to " << value << std::endl;
      return value;
    };
    auto b_functor = [this, &index](double v) {
      if (v < 0) v = 0;
      if (v > 1.0) v = 1.0;
      Eigen::VectorXd coeffcient = this->f_coefficient(v);
      Eigen::VectorXd param = this->_b_params.block<15, 1>(index * 15, 0);
      double value = param.dot(coeffcient);
      std::cout << index << " B : " << v << " map to " << value << std::endl;
      return value;
    };
    _interface->ApplyTransform(index, r_functor, g_functor, b_functor);
  }
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
      q(node_index * 256 + i) = -2.0 * v;
    }
  }
  _range_quadratic_matrix = p.transpose() * p;
  _range_linear_vector = q.transpose() * p;
}

void ColorTransform::GenerateHardLimitedFormula() {
  _nonequal_formula_matrix = Eigen::MatrixXd::Zero(
      10 * (_node_size - _fixed_image_size), 15 * _node_size);
  _nonequal_formula_constant =
      Eigen::VectorXd::Zero(10 * (_node_size - _fixed_image_size));
  auto image_ids = _interface->GetIndex();
  auto fixed_image_ids = _interface->GetFixedIndex();
  std::set<size_t> fixed_image_ids_set;
  for (size_t id : fixed_image_ids) {
    fixed_image_ids_set.insert(id);
  }
  size_t count = 0;
  for (size_t id : image_ids) {
    if (fixed_image_ids_set.find(id) == fixed_image_ids_set.end()) {
      size_t index = _id_to_idx[id];
      for (size_t j = 0; j < 5; j++) {
        _nonequal_formula_matrix.block<1, 15>(count * 10 + j * 2 + 0,
                                              15 * index) =
            f_jacobian_coefficient(0.2 * j + 0.1).transpose();
        _nonequal_formula_constant(count * 10 + j * 2 + 0) = -0.2;
        _nonequal_formula_matrix.block<1, 15>(count * 10 + j * 2 + 1,
                                              15 * index) =
            -f_jacobian_coefficient(0.2 * j + 0.1).transpose();
        _nonequal_formula_constant(count * 10 + j * 2 + 1) = 5.0;
      }
      //_nonequal_formula_matrix.block<1, 15>(count * 11 + 10, 15 * index) =
      -f_coefficient(0.0);
      //_nonequal_formula_constant(count * 11 + 10, 0) = 0.0;
      count++;
    }
  }
  /*
   _nonequal_formula_matrix = Eigen::MatrixXd::Zero(1, 15 * _node_size);
   _nonequal_formula_constant = Eigen::VectorXd::Zero(1);
   _nonequal_formula_matrix.block<1, 15>(0, 15) =
       f_jacobian_coefficient(0.1).transpose();
   _nonequal_formula_constant(0, 0) = -0.2;
   */
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

void ColorTransform::GenerateSplineFormula() {
  std::cout << "GenerateSplineformula" << std::endl;
  _spline_equal_matrix = Eigen::MatrixXd::Zero(
      8 * (_node_size - _fixed_image_size), 15 * _node_size);
  _spline_equal_constant =
      Eigen::VectorXd::Zero(8 * (_node_size - _fixed_image_size));
  auto image_ids = _interface->GetIndex();
  auto fixed_image_ids = _interface->GetFixedIndex();
  std::set<size_t> fixed_image_ids_set;
  for (size_t id : fixed_image_ids) {
    fixed_image_ids_set.insert(id);
  }
  size_t count = 0;
  for (size_t id : image_ids) {
    if (fixed_image_ids_set.find(id) == fixed_image_ids_set.end()) {
      size_t index = _id_to_idx[id];
      for (size_t j = 0; j < 4; j++) {
        _spline_equal_matrix(count * 8 + j * 2 + 0, index * 15 + j * 3 + 0) =
            1.0;
        _spline_equal_matrix(count * 8 + j * 2 + 0, index * 15 + j * 3 + 1) =
            0.2;
        _spline_equal_matrix(count * 8 + j * 2 + 0, index * 15 + j * 3 + 2) =
            0.04;
        _spline_equal_matrix(count * 8 + j * 2 + 0, index * 15 + j * 3 + 3) =
            -1.0;

        _spline_equal_matrix(count * 8 + j * 2 + 1, index * 15 + j * 3 + 1) =
            1.0;
        _spline_equal_matrix(count * 8 + j * 2 + 1, index * 15 + j * 3 + 2) =
            0.4;
        _spline_equal_matrix(count * 8 + j * 2 + 1, index * 15 + j * 3 + 4) =
            -1.0;
      }
      count++;
    }
  }
  std::cout << "GenerateSplineformula Done" << std::endl;
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
  _r_params = Eigen::VectorXd::Zero(15 * _node_size);
  bool res = MinimumQuadraticProgram(_r_quadratic_matrix, _r_params);
  if (!res) {
    std::cerr << "Quadratic Problem Can not be Solved." << std::endl;
    exit(1);
  } else {
    std::cout << "R X : ";
    for (size_t i = 0; i < 15; i++) {
      std::cout << _r_params(i, 0) << " ";
    }
    std::cout << std::endl;
    for (size_t i = 15; i < 30; i++) {
      std::cout << _r_params(i, 0) << " ";
    }
    std::cout << std::endl;
  }
  _g_params = Eigen::VectorXd::Zero(15 * _node_size);
  res &= MinimumQuadraticProgram(_g_quadratic_matrix, _g_params);
  if (!res) {
    std::cerr << "Quadratic Problem Can not be Solved." << std::endl;
    exit(1);
  } else {
    std::cout << "G X : ";
    for (size_t i = 0; i < 15; i++) {
      std::cout << _r_params(i, 0) << " ";
    }
    std::cout << std::endl;
    for (size_t i = 15; i < 30; i++) {
      std::cout << _r_params(i, 0) << " ";
    }
    std::cout << std::endl;
  }
  _b_params = Eigen::VectorXd::Zero(15 * _node_size);
  res &= MinimumQuadraticProgram(_b_quadratic_matrix, _b_params);
  if (!res) {
    std::cerr << "Quadratic Problem Can not be Solved." << std::endl;
    exit(1);
  } else {
    std::cout << "G X : ";
    for (size_t i = 0; i < 15; i++) {
      std::cout << _r_params(i, 0) << " ";
    }
    std::cout << std::endl;
    for (size_t i = 15; i < 30; i++) {
      std::cout << _r_params(i, 0) << " ";
    }
    std::cout << std::endl;
  }
}
bool ColorTransform::MinimumQuadraticProgram(
    const Eigen::Ref<const Eigen::MatrixXd> &quadratic_matrix,
    Eigen::Ref<Eigen::VectorXd> solution_param) {
  Eigen::MatrixXd QQ = quadratic_matrix + 0.03 * this->_range_quadratic_matrix;
  quadprogpp::Matrix<double> Q = EigenMatrixToQuadProgMatrix(QQ);
  quadprogpp::Vector<double> g =
      EigenVectorToQuadProgVector(this->_range_linear_vector);
  std::cout << "concat mat" << std::endl;
  std::cout << "_equal_formula_matrix : " << _equal_formula_matrix.rows() << " "
            << _equal_formula_matrix.cols() << std::endl;
  std::cout << "_spline_equal_matrix : " << _spline_equal_matrix.rows() << " "
            << _spline_equal_matrix.cols() << std::endl;
  Eigen::MatrixXd temp_equal_matrix(
      _equal_formula_matrix.rows() + _spline_equal_matrix.rows(),
      15 * _node_size);
  temp_equal_matrix << _equal_formula_matrix, _spline_equal_matrix;
  Eigen::VectorXd temp_equal_constant(_equal_formula_constant.rows() +
                                      _spline_equal_constant.rows());

  temp_equal_constant << _equal_formula_constant, _spline_equal_constant;
  quadprogpp::Matrix<double> CE =
      EigenMatrixToQuadProgMatrix(temp_equal_matrix);

  quadprogpp::Vector<double> ce0 =
      EigenVectorToQuadProgVector(temp_equal_constant);
  quadprogpp::Matrix<double> CI =
      EigenMatrixToQuadProgMatrix(_nonequal_formula_matrix);
  // CI.resize(15 * _node_size, 0);
  quadprogpp::Vector<double> ci0 =
      EigenVectorToQuadProgVector(_nonequal_formula_constant);
  // ci0.resize(0);

  quadprogpp::Vector<double> x;
  x.resize(15 * _node_size);
  double error = quadprogpp::solve_quadprog(Q, g, CE, ce0, CI, ci0, x);
  std::cout << "Error : " << error << std::endl;
  if (error == INFINITY) {
    return false;
  }
  Eigen::VectorXd params = Eigen::VectorXd::Zero(15 * _node_size);
  std::cout << "solution : " << solution_param.rows() << " X "
            << solution_param.cols() << std::endl;
  for (size_t i = 0; i < 15 * _node_size; i++) {
    params(i, 0) = x[i];
    solution_param(i, 0) = x[i];
  }
  // solution_param = params;
  return true;
}

quadprogpp::Matrix<double> ColorTransform::EigenMatrixToQuadProgMatrix(
    const Eigen::Ref<const Eigen::MatrixXd> &matrix) {
  size_t ROW = matrix.rows();
  size_t COL = matrix.cols();
  quadprogpp::Matrix<double> res;
  if (COL == 0) {
    std::cerr << "Eigen Matrix's column equal to zero " << __FILE__ << " "
              << __LINE__ << std::endl;
  }
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
    return res;
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
Eigen::VectorXd ColorTransform::f_jacobian_coefficient(double v) {
  Eigen::VectorXd res = Eigen::VectorXd::Zero(15);
  if (v < 0.0) {
    std::cerr << "Waring: f_jacobian_coefficient recvive a value less then Zero"
              << std::endl;
    return res;
  }
  if (v > 1.0) {
    std::cerr << "Waring: f_coefficient recvive a value more then One"
              << std::endl;
    return res;
  }

  int index = static_cast<int>(v / 0.2);
  index = index > 4 ? 4 : index;
  res(index * 3 + 1, 0) = 1.0;
  res(index * 3 + 2, 0) = 2 * (v - 0.2 * index);
  return res;
}