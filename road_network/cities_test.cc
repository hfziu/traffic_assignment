//
// Created by Xiaofei ZHAO on 3/9/2023.
//

#include <gtest/gtest.h>

#include "road_network/assignment.h"
#include "road_network/graph.h"
#include "util/csv_reader.h"

namespace road_network {

TEST(TrafficAssignmentTest, SanFrancisco) {
  // TODO: read number of nodes from info file
  auto graph = Graph(2982);
  auto link_file = util::GetFileContents(
      "../data/20_honolulu/base_info/honolulu_link.csv");
  auto link_lines = util::SplitLines(link_file, true);  // skip header
  for (auto&& line : link_lines) {
    ASSERT_TRUE(util::AddLinkFromLine(graph, line));
  }

  auto demand_file = util::GetFileContents(
      "../data/20_honolulu/base_info/honolulu_od.csv");
  auto demand_lines = util::SplitLines(demand_file, true);  // skip header
  for (auto&& line : demand_lines) {
    ASSERT_TRUE(util::AddDemandFromLine(graph, line));
  }

  // Traffic assignment
  MSA msa;
  msa.Solve(graph, 100, 1e-3);
}

}  // namespace road_network