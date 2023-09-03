//
// Created by Xiaofei ZHAO on 29/8/2023.
//

#ifndef TRAFFIC_ASSIGNMENT_ROAD_NETWORK_MATRIX_H_
#define TRAFFIC_ASSIGNMENT_ROAD_NETWORK_MATRIX_H_

#include <map>

namespace road_network {

template <typename T>
concept arithmetic = std::is_arithmetic_v<T>;

// road_network::Matrix is a helper class to maintain the adjacency matrix.
// Each element of the matrix is a road_network::Link.
template <typename T>
class Matrix {
 public:
  Matrix() = default;
  virtual ~Matrix() = default;

  // Data access
  virtual T& operator()(int row, int col) = 0;
  virtual void Resize(int row, int col) = 0;
  virtual const T& at(int row, int col) const = 0;  // read only

  // Data manipulation
  virtual void Clear() = 0;
};  // class Matrix

// A sparse matrix intended to store arbitrary data.
template <typename T>
class SparseMatrix : public Matrix<T> {
 public:
  SparseMatrix() = default;
  ~SparseMatrix() override = default;

  // Constructor
  SparseMatrix(int row, int col) : n_row_(row), n_col_(col) {
    data_ = std::map<int, std::map<int, T>>();
  }
  // Copy constructor
  SparseMatrix(const SparseMatrix& other) {
    n_row_ = other.n_row_;
    n_col_ = other.n_col_;
    data_ = other.data_;
  };
  // Move constructor
  SparseMatrix(SparseMatrix&& other) noexcept {
    n_row_ = other.n_row_;
    n_col_ = other.n_col_;
    data_ = std::move(other.data_);
  };

  // Data access
  T& operator()(int row, int col) override {  // read/write
    if (data_.find(row) == data_.end()) {
      data_[row] = std::map<int, T>();
    }
    return data_[row][col];
  };

  void Resize(int row, int col) override {
    if (row < n_row_ || col < n_col_) {
      throw std::runtime_error("Cannot shrink the matrix");
    }
    n_row_ = row;
    n_col_ = col;
  };

  const T& at(int row, int col) const override {  // read only
    if (data_.find(row) == data_.end() ||
        data_.at(row).find(col) == data_.at(row).end()) {
      throw std::out_of_range("location not found in the matrix.");
    }
    return data_.at(row).at(col);
  };

  // Iterators
  auto begin() { return data_.begin(); }
  auto end() { return data_.end(); }

  // Data manipulation
  void Clear() override { data_.clear(); }

 protected:
  int n_row_{};
  int n_col_{};
  std::map<int, std::map<int, T>> data_;
};

template <arithmetic T>
class SparseMatrixArithmetic : public SparseMatrix<T> {
 public:
  SparseMatrixArithmetic() = default;
  ~SparseMatrixArithmetic() override = default;

  // Constructor
  SparseMatrixArithmetic(int row, int col, T zero = T())
      : SparseMatrix<T>(row, col), zero_(zero) {}
  // Copy constructor
  SparseMatrixArithmetic(const SparseMatrixArithmetic& other) {
    SparseMatrix<T>::n_row_ = other.n_row_;
    SparseMatrix<T>::n_col_ = other.n_col_;
    SparseMatrix<T>::data_ = other.data_;
    zero_ = other.zero_;
  };
  // Move constructor
  SparseMatrixArithmetic(SparseMatrixArithmetic&& other) noexcept {
    SparseMatrix<T>::n_row_ = other.n_row_;
    SparseMatrix<T>::n_col_ = other.n_col_;
    SparseMatrix<T>::data_ = std::move(other.data_);
    zero_ = other.zero_;
  };

  // Read only access
  const T& at(int row, int col) const override {  // read only
    if (SparseMatrix<T>::data_.find(row) == SparseMatrix<T>::data_.end() ||
        SparseMatrix<T>::data_.at(row).find(col) ==
            SparseMatrix<T>::data_.at(row).end()) {
      return zero_;
    }
    return SparseMatrix<T>::data_.at(row).at(col);
  };

  const T& get_zero() const { return zero_; }

 private:
  T zero_{};
};

}  // namespace road_network

#endif  // TRAFFIC_ASSIGNMENT_ROAD_NETWORK_MATRIX_H_
