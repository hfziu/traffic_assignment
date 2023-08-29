//
// Created by Xiaofei ZHAO on 29/8/2023.
//

#ifndef TRAFFIC_ASSIGNMENT_ROAD_NETWORK_MATRIX_H_
#define TRAFFIC_ASSIGNMENT_ROAD_NETWORK_MATRIX_H_

namespace road_network {

// road_network::Matrix is a helper class to maintain the adjacency matrix.
// Each element of the matrix is a road_network::Link.
template <typename T>
class Matrix {
 public:
  Matrix() = default;
  ~Matrix() = default;

  Matrix(int row, int col) { Resize(row, col); }

  // move constructor
  Matrix(Matrix&& other) noexcept : data_(std::move(other.data_)) {}

  T& operator()(int row, int col) { return data_[row * n_col_ + col]; }

  void Resize(int row, int col) {
    n_row_ = row;
    n_col_ = col;
    data_.resize(n_row_ * n_col_);
  }

  void Clear() { data_.clear(); }

 private:
  int n_row_{}, n_col_{};
  std::vector<T> data_{};
};  // class Matrix

}  // namespace road_network

#endif  // TRAFFIC_ASSIGNMENT_ROAD_NETWORK_MATRIX_H_
