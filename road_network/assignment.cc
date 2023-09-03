//
// Created by Xiaofei ZHAO on 30/8/2023.
//

#include "road_network/assignment.h"

#include <chrono>

namespace road_network {

void MSA::Solve(Graph& graph, int max_iter, double eps) {
  // Timer
  auto start = std::chrono::steady_clock::now();
  // initialize the origin nodes from demand matrix
  NodeSet origins{};
  for (const auto& [k, v] : graph.demands_) {
    origins.insert(k);
  }

  // initialize the link costs using zero flow
  graph.link_flows_.setZero();
  graph.UpdateLinkCosts();
  graph.UpdateShortestPathMany(origins);
  double aec = std::numeric_limits<double>::max();
  AssignAON(graph);  // result is saved in graph.target_link_flows_.
  graph.link_flows_ = graph.target_link_flows_;
#ifndef NDEBUG
  std::cout << graph.link_flows_ << std::endl;
  std::cout << graph.link_costs_.at(0, 3) << std::endl;
#endif
  int iter = 1;

  while (iter < max_iter) {
    graph.UpdateLinkCosts();
    graph.UpdateShortestPathMany(origins);
    aec = graph.ComputeAEC();
    std::cout << "Iteration " << iter << ": AEC = " << aec << " | ";
    auto end = std::chrono::steady_clock::now();
    std::cout
        << "Time elapsed: "
        << std::chrono::duration_cast<std::chrono::seconds>(end - start).count()
        << " sec" << std::endl;
    start = end;
#ifndef NDEBUG
    if (iter < 10 || iter % 10 == 0) {
      // link travel time
      std::cout << "Link travel time: ";
      std::cout << "t = [ ";
      for (auto& from_node : graph.links_) {
        for (auto& link : from_node.second) {
          std::cout << link.second.TravelTime(
                           graph.link_flows_.coeff(from_node.first, link.first))
                    << " ";
        }
      }
      std::cout << "]" << std::endl;
    }
#endif
    if (aec < eps) {
      break;
    }
    AssignAON(graph);
    graph.link_flows_ = iter / (iter + 1.0) * graph.link_flows_ +
                        1.0 / (iter + 1.0) * graph.target_link_flows_;
    ++iter;
  }
}

void MSA::AssignAON(Graph& graph) {
  // traverse all the demands
  // r: origin node (s_map is a map of destination nodes from r)
  // s: destination node
  // d: demand between r and s
  graph.target_link_flows_.setZero();  // initialize to zero
  for (const auto& [r, s_map] : graph.demands_) {
    for (const auto& [s, d] : s_map) {
      // update the link flow on the shortest path from r to s
      auto& path = graph.shortest_path_(r, s).second;
      if (path.size() > 1) {
        for (int i = 0; i < path.size() - 1; ++i) {
          graph.target_link_flows_.coeffRef(path[i], path[i + 1]) += d;
        }
      }
    }
  }
}

}  // namespace road_network