//
// Created by Xiaofei ZHAO on 3/9/2023.
//

#include <gtest/gtest.h>

#include "road_network/assignment.h"
#include "road_network/graph.h"
#include "util/csv_reader.h"

namespace road_network {

TEST(TrafficAssignmentTest, San_francisco) {
  // TODO: read number of nodes from info file

  auto graph = Graph(4986);
  auto link_file = util::GetFileContents(
      "../data/01_san_francisco/base_info/san_francisco_link_new.csv");

  auto link_lines = util::SplitLines(link_file, true);  // skip header
  for (auto&& line : link_lines) {
    ASSERT_TRUE(util::AddLinkFromLine(graph, line));
  }

  auto demand_file = util::GetFileContents(
      "../data/01_san_francisco/base_info/san_francisco_od.csv");
  auto demand_lines = util::SplitLines(demand_file, true);  // skip header
  for (auto&& line : demand_lines) {
    ASSERT_TRUE(util::AddDemandFromLine(graph, line));
  }

  // Traffic assignment
  MSA msa;
  msa.Solve(graph, 100, 1e-4);

  // write the link flows to file
  util::WriteLinkFlow(
      graph, "../data/01_san_francisco/base_info/san_francisco_link_flow.csv");

  // find the link with the largest flow
  auto max_flow = 0.0;
  std::pair<NodeID, NodeID> max_flow_link;
  for (auto& from_node : graph.GetLinks()) {
    for (auto& to_node : from_node.second) {
      auto flow = graph.GetLinkFlows().coeff(from_node.first, to_node.first);
      if (flow > max_flow) {
        max_flow = flow;
        max_flow_link = std::make_pair(from_node.first, to_node.first);
      }
    }
  }
  // get the link cost
  auto max_flow_link_cost = graph.GetLinks()
                                .at(max_flow_link.first, max_flow_link.second)
                                .TravelTime(max_flow);

  auto multiplier = std::vector<double>{0.1, 0.5, 0.9};
  for (auto mul : multiplier) {
    // alter the link performance function so that it always returns a constant
    // that is mul times the original cost
    graph.SetLinkPerformanceFunction(
        max_flow_link.first, max_flow_link.second,
        [mul, max_flow_link_cost](double flow) { return mul * max_flow_link_cost; });

    // re-assign the traffic
    std::cout << "Re-assigning the traffic after alter the link cost with "
                 "maximum flow ..."
              << std::endl;
    msa.Solve(graph, 100, 1e-4);

    // write the link flows to file
    auto file_name = "../data/01_san_francisco/base_info/san_francisco_link_flow_" +
                     std::to_string(mul) + "cost.csv";
    util::WriteLinkFlow(graph,
                        file_name);
  }
}

}  // namespace road_network