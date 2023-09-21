//
// Created by Xiaofei ZHAO on 3/9/2023.
//

#include <gtest/gtest.h>

#include "road_network/assignment.h"
#include "road_network/graph.h"
#include "util/csv_reader.h"

namespace road_network {

// load link flows from file
void inline LoadLinkFlowsFromFile(Graph& graph) {
  auto link_flow_file = util::GetFileContents(
      "../data/01_san_francisco/base_info/san_francisco_link_flow_50.csv");
  auto link_flow_lines = util::SplitLines(link_flow_file, true);  // skip header
  for (auto&& line : link_flow_lines) {
    ASSERT_TRUE(util::UpdateLinkFlowFromLine(graph, line));
  }
}

TEST(TrafficAssignmentTest, Minneapolis) {
  // TODO: read number of nodes from info file

  auto graph = Graph(4986);
  auto link_file = util::GetFileContents(
      "../data/01_san_francisco/base_info/san_francisco_link.csv");

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

  // traffic assignment
  MSA msa;

  // {
  //   // COMMENT OUT IF YOU USE PRE-COMPUTED LINK FLOWS
  //   // solve
  //   msa.Solve(graph, 50, 1e-2);

  //   // write the link flows to file.
  //   util::WriteLinkFlow(
  //       graph,
  //       "../data/01_san_francisco/base_info/san_francisco_link_flow_50.csv");
  //   // COMMENT OUT IF YOU USE PRE-COMPUTED LINK FLOWS
  // }

  // load link flows from file
  LoadLinkFlowsFromFile(graph);

  auto multiplier = std::vector<double>{0.9, 1.1};

  // find the link with the largest link cost
  // auto max_cost = 0.0;
  // std::pair<NodeID, NodeID> max_cost_link;
  // for (auto& from_node : graph.GetLinks()) {
  //   if (graph.GetNodeName(from_node.first).size() > 7) continue;
  //   for (auto& to_node : from_node.second) {
  //     if (graph.GetNodeName(to_node.first).size() > 7) continue;
  //     auto flow = graph.GetLinkFlows().coeff(from_node.first, to_node.first);
  //     auto cost =
  //         graph.GetLinks().at(from_node.first,
  //         to_node.first).TravelTime(flow);
  //     if (cost > max_cost) {
  //       max_cost = cost;
  //       max_cost_link = std::make_pair(from_node.first, to_node.first);
  //     }
  //   }
  // }
  // auto max_cost_link_cost = max_cost;
  // std::cout << "max cost link: from " <<
  // graph.GetNodeName(max_cost_link.first)
  //           << " to " << graph.GetNodeName(max_cost_link.second) <<
  //           std::endl;
  // std::cout << "max_cost_link_cost: " << max_cost_link_cost << std::endl;

  // for (auto mul : multiplier) {
  //   // backup the link performance function
  //   auto backup_link_performance_function =
  //       graph.GetLinks()
  //           .at(max_cost_link.first, max_cost_link.second)
  //           .GetPerformanceFunction();
  //   // alter the link performance function so that it always returns a
  //   constant
  //   // that is mul times the original cost
  //   graph.SetLinkPerformanceFunction(max_cost_link.first,
  //   max_cost_link.second,
  //                                    [mul, max_cost_link_cost](double flow) {
  //                                      return mul * max_cost_link_cost;
  //                                    });

  //   // re-assign the traffic
  //   std::cout << "Re-assigning the traffic after alter the link cost with "
  //                "maximum COST ..."
  //             << std::endl;
  //   msa.Solve(graph, 50, 1e-2);

  //   // restore the link performance function
  //   graph.SetLinkPerformanceFunction(max_cost_link.first,
  //   max_cost_link.second,
  //                                    backup_link_performance_function);

  //   // write the link flows to file
  //   auto file_name =
  //       "../data/01_san_francisco/base_info/"
  //       "san_francisco_link_flow_max_cost_link_50_" +
  //       std::to_string(mul) + "cost.csv";
  //   util::WriteLinkFlow(graph, file_name);
  // }

  // find the link with the largest voc
  // LoadLinkFlowsFromFile(graph);
  // auto max_voc = 0.0;
  // std::pair<NodeID, NodeID> max_voc_link;
  // auto max_voc_link_flow =
  //     0.0;  // flow of the link with largest VOC in the network
  // for (auto& from_node : graph.GetLinks()) {
  //   if (graph.GetNodeName(from_node.first).size() > 7) continue;
  //   for (auto& to_node : from_node.second) {
  //     if (graph.GetNodeName(to_node.first).size() > 7) continue;
  //     auto flow = graph.GetLinkFlows().coeff(from_node.first, to_node.first);
  //     auto capacity =
  //         graph.GetLinks().at(from_node.first, to_node.first).GetCapacity();
  //     auto voc = flow / capacity;
  //     if (voc > max_voc) {
  //       max_voc = voc;
  //       max_voc_link_flow = flow;
  //       max_voc_link = std::make_pair(from_node.first, to_node.first);
  //     }
  //   }
  // }

  // auto max_voc_link_cost = graph.GetLinks()
  //                              .at(max_voc_link.first, max_voc_link.second)
  //                              .TravelTime(max_voc_link_flow);

  // std::cout << "max voc link: from " << graph.GetNodeName(max_voc_link.first)
  //           << " to " << graph.GetNodeName(max_voc_link.second) << std::endl;
  // std::cout << "max_voc_link_cost: " << max_voc_link_cost << std::endl;

  // for (auto mul : multiplier) {
  //   // backup the link performance function
  //   auto backup_link_performance_function =
  //       graph.GetLinks()
  //           .at(max_voc_link.first, max_voc_link.second)
  //           .GetPerformanceFunction();
  //   // alter the link performance function so that it always returns a
  //   constant
  //   // that is mul times the original cost
  //   graph.SetLinkPerformanceFunction(max_voc_link.first, max_voc_link.second,
  //                                    [mul, max_voc_link_cost](double flow) {
  //                                      return mul * max_voc_link_cost;
  //                                    });

  //   // re-assign the traffic
  //   std::cout << "Re-assigning the traffic after alter the link cost with "
  //                "maximum VOC ..."
  //             << std::endl;
  //   msa.Solve(graph, 50, 1e-2);

  //   // restore the link performance function
  //   graph.SetLinkPerformanceFunction(max_voc_link.first, max_voc_link.second,
  //                                    backup_link_performance_function);

  //   // write the link flows to file
  //   auto file_name =
  //       "../data/01_san_francisco/base_info/"
  //       "san_francisco_link_flow_max_voc_link_50_" +
  //       std::to_string(mul) + "cost.csv";
  //   util::WriteLinkFlow(graph, file_name);
  // }

  // find the link with largest partial derivative of TTT with respect to flow
  LoadLinkFlowsFromFile(graph);
  auto max_partial_derivative = 0.0;
  std::pair<NodeID, NodeID> max_partial_derivative_link;
  auto max_partial_derivative_link_flow =
      0.0;  // flow of the link with largest partial derivative in the network
  for (auto& from_node : graph.GetLinks()) {
    if (graph.GetNodeName(from_node.first).size() > 7) continue;
    for (auto& to_node : from_node.second) {
      if (graph.GetNodeName(to_node.first).size() > 7) continue;
      auto flow = graph.GetLinkFlows().coeff(from_node.first, to_node.first);
      auto voc =
          flow /
          graph.GetLinks().at(from_node.first, to_node.first).GetCapacity();
      auto free_time =
          graph.GetLinks().at(from_node.first, to_node.first).TravelTime(0);
      auto partial_derivative = free_time * (1.0 + 1.5 * voc * voc);
      if (partial_derivative > max_partial_derivative) {
        max_partial_derivative = partial_derivative;
        max_partial_derivative_link_flow = flow;
        max_partial_derivative_link =
            std::make_pair(from_node.first, to_node.first);
      }
    }
  }

  auto max_partial_derivative_link_cost =
      graph.GetLinks()
          .at(max_partial_derivative_link.first,
              max_partial_derivative_link.second)
          .TravelTime(max_partial_derivative_link_flow);

  std::cout << "max partial derivative link: from "
            << graph.GetNodeName(max_partial_derivative_link.first) << " to "
            << graph.GetNodeName(max_partial_derivative_link.second)
            << std::endl;
  std::cout << "max_partial_derivative_link_cost: "
            << max_partial_derivative_link_cost << std::endl;

  for (auto mul : multiplier) {
    // backup the link performance function
    auto backup_link_performance_function =
        graph.GetLinks()
            .at(max_partial_derivative_link.first,
                max_partial_derivative_link.second)
            .GetPerformanceFunction();
    // alter the link performance function so that it always returns a constant
    // that is mul times the original cost
    graph.SetLinkPerformanceFunction(
        max_partial_derivative_link.first, max_partial_derivative_link.second,
        [mul, max_partial_derivative_link_cost](double flow) {
          return mul * max_partial_derivative_link_cost;
        });

    // re-assign the traffic
    std::cout << "Re-assigning the traffic after alter the link cost with "
                 "maximum partial derivative ..."
              << std::endl;
    msa.Solve(graph, 50, 1e-2);

    // restore the link performance function
    graph.SetLinkPerformanceFunction(max_partial_derivative_link.first,
                                     max_partial_derivative_link.second,
                                     backup_link_performance_function);

    // write the link flows to file
    auto file_name =
        "../data/01_san_francisco/base_info/"
        "san_francisco_link_flow_max_partial_derivative_link_50_" +
        std::to_string(mul) + "cost.csv";
    util::WriteLinkFlow(graph, file_name);
  }
}

}  // namespace road_network