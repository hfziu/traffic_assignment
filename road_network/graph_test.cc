//
// Created by Xiaofei ZHAO on 29/8/2023.
//

#include "road_network/graph.h"

#include <gtest/gtest.h>

#include "road_network/link_performance.h"

namespace road_network {

TEST(GraphTest, SimpleGraph) {
  Graph g(3);
  g.AddLink("A", "B", 100, 100, 100, 1, 1, BPR(1, 100, 0.15, 4));
  g.AddLink("A", "C", 200, 120, 100, 2, 1, BPR(1, 200, 0.15, 4));
  g.AddLink("B", "C", 100, 100, 100, 1, 1, BPR(1, 100, 0.15, 4));
  ASSERT_EQ(g.GetLink("A", "C").TravelTime(200), 1.15);  // TODO: fix: comparing double values
}

TEST(GraphTest, SmallNetwork) {
  // This is the example in Chapter 6 of the book Transportation Network Analysis
  // https://sboyles.github.io/book.pdf
  Graph g(6);


}

}  // namespace road_network