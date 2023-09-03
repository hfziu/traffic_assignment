//
// Created by Xiaofei ZHAO on 2/9/2023.
//

#include <gtest/gtest.h>
#include <road_network/matrix.h>

namespace road_network {

TEST(MatrixTest, SimpleSparseMatrix) {
  SparseMatrix<double> m(3, 3);
  m(0, 0) = 1.0;
  m(0, 1) = 2.0;
  m(1, 0) = 3.0;
  m(1, 1) = 4.0;
  ASSERT_EQ(m(0, 0), 1.0);
  ASSERT_EQ(m(0, 1), 2.0);
  ASSERT_EQ(m(1, 0), 3.0);
  ASSERT_EQ(m(1, 1), 4.0);
  ASSERT_EQ(m(0, 2), 0.0);
  ASSERT_EQ(m(2, 0), 0.0);
  ASSERT_EQ(m(2, 2), 0.0);

  // Iteration
  for (auto& row : m) {
    for (auto& col : row.second) {
      std::cout << row.first << ", " << col.first << ", " << col.second
                << std::endl;
    }
  }
}

}  // namespace road_network