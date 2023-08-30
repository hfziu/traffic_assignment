//
// Created by Xiaofei ZHAO on 30/8/2023.
//

#ifndef TRAFFIC_ASSIGNMENT_ROAD_NETWORK_ASSIGNMENT_H_
#define TRAFFIC_ASSIGNMENT_ROAD_NETWORK_ASSIGNMENT_H_

#include "road_network/graph.h"

namespace road_network {

class Assignment {
 public:
  Assignment() = default;
  ~Assignment() = default;

  // Solve the traffic assignment problem, update corresponding matrices in the
  // graph.
  virtual void Solve(Graph& graph, int max_iter, double eps) = 0;
};

class MSA : public Assignment {
 public:
  MSA() = default;
  ~MSA() = default;

  void Solve(Graph& graph, int max_iter, double eps) override;

  // All-or-Nothing assignment corresponding to the current link costs.
  // The result is saved in graph.target_link_flows_.
  static void AssignAON(Graph& graph);
};

}  // namespace road_network

#endif  // TRAFFIC_ASSIGNMENT_ROAD_NETWORK_ASSIGNMENT_H_
