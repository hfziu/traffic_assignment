//
// Created by Xiaofei ZHAO on 29/8/2023.
//

#include "road_network/graph.h"

#include <gtest/gtest.h>

#include "road_network/assignment.h"
#include "road_network/link_performance.h"

namespace road_network {

TEST(GraphTest, SimpleGraph) {
  Graph g(3);
  g.AddLink("A", "B", 100, 100, 100, 1, 1, BPR(1, 100, 0.15, 4));
  g.AddLink("A", "C", 200, 120, 100, 2, 1, BPR(1, 200, 0.15, 4));
  g.AddLink("B", "C", 100, 100, 100, 1, 1, BPR(1, 100, 0.15, 4));
  ASSERT_EQ(g.GetLink("A", "C")->TravelTime(200),
            1.15);  // TODO: fix: comparing double values
}

TEST(GraphTest, SmallNetwork) {
  // This is the example in Chapter 6 of the book Transportation Network
  // Analysis https://sboyles.github.io/book.pdf
  LinkPerformanceFunction f = [](double x) -> double { return 10 + x / 100; };
  Graph g(6);
  g.AddLink("1", "3", 0, 0, 0, 0, 0, f);
  g.AddLink("1", "5", 0, 0, 0, 0, 0, f);
  g.AddLink("5", "6", 0, 0, 0, 0, 0, f);
  g.AddLink("6", "3", 0, 0, 0, 0, 0, f);
  g.AddLink("2", "5", 0, 0, 0, 0, 0, f);
  g.AddLink("6", "4", 0, 0, 0, 0, 0, f);
  g.AddLink("2", "4", 0, 0, 0, 0, 0, f);

  g.AddDemand("1", "3", 5000);
  g.AddDemand("2", "4", 10000);

  ASSERT_EQ(g.GetLink("1", "3")->TravelTime(5000), 60);
  ASSERT_EQ(g.GetLink("2", "4")->TravelTime(10000), 110);
}

TEST(GraphTest, MSAExample) {
  // Build the network
  // This is the example in Chapter 6 of the book Transportation Network
  // Analysis https://sboyles.github.io/book.pdf
  LinkPerformanceFunction f = [](double x) -> double { return 10 + x / 100; };
  Graph g(6);
  g.AddLink("1", "3", 0, 0, 0, 0, 0, f);
  g.AddLink("1", "5", 0, 0, 0, 0, 0, f);
  g.AddLink("5", "6", 0, 0, 0, 0, 0, f);
  g.AddLink("6", "3", 0, 0, 0, 0, 0, f);
  g.AddLink("2", "5", 0, 0, 0, 0, 0, f);
  g.AddLink("6", "4", 0, 0, 0, 0, 0, f);
  g.AddLink("2", "4", 0, 0, 0, 0, 0, f);

  g.AddDemand("1", "3", 5000);
  g.AddDemand("2", "4", 10000);

  // MSA solver
  MSA msa;
  msa.Solve(g, 100, 0.01);
}

}  // namespace road_network