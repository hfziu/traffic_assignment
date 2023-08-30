//
// Created by Xiaofei ZHAO on 30/8/2023.
//

#include "road_network/assignment.h"

namespace road_network {

void MSA::Solve(Graph& graph, int max_iter, double eps) {
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
  int iter = 1;

  while (iter < max_iter) {
    graph.UpdateLinkCosts();
    graph.UpdateShortestPathMany(origins);
    aec = graph.ComputeAEC();
#ifndef NDEBUG
    if (iter < 10 || iter % 10 == 0) {
      std::cout << "Iteration " << iter << ": AEC = " << aec << std::endl;
      // link travel time
      std::cout << "Link travel time: ";
      std::cout << "t = [ ";
      for (int i = 0; i < graph.n_nodes_; ++i) {
        for (int j = 0; j < graph.n_nodes_; ++j) {
          if (graph.adjacency_matrix_(i, j).has_value()) {
            std::cout << graph.adjacency_matrix_(i, j)->TravelTime(
                             graph.link_flows_(i, j))
                      << " ";
          }
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
          graph.target_link_flows_(path[i], path[i + 1]) += d;
        }
      }
    }
  }
}

}  // namespace road_network