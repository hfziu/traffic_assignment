//
// Created by Xiaofei ZHAO on 29/8/2023.
//

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <gtest/gtest.h>

// Demonstrate some basic assertions.
TEST(HelloTest, BasicAssertions) {
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  // Expect equality.
  EXPECT_EQ(7 * 6, 42);
}

// Eigen example.
TEST(EigenTest, BasicMatrixOperations) {
  Eigen::MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;
  ASSERT_EQ(m(1,1), 1.5);
}

// Sparse matrix example.
TEST(EigenTest, SparseMatrix) {
  Eigen::SparseMatrix<double> m(3,3);
  m.insert(0,0) = 3;
  m.insert(1,0) = 2.5;
  m.insert(0,1) = -1;
  m.insert(1,1) = m.coeff(1,0) + m.coeff(0,1);
  m.coeffRef(2,1) = 0.0;
  std::cout << m << std::endl;
  ASSERT_EQ(m.coeff(1,1), 1.5);
  std::cout << m.coeff(2,1) << std::endl;
  // What if we print an implicitly zero element?
  std::cout << m.coeff(2,2) << std::endl;
}