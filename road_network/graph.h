//
// Created by Xiaofei ZHAO on 29/8/2023.
//

#ifndef TRAFFIC_ASSIGNMENT_ROAD_NETWORK_GRAPH_H_
#define TRAFFIC_ASSIGNMENT_ROAD_NETWORK_GRAPH_H_

#include <Eigen/Dense>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "road_network/format.h"
#include "road_network/matrix.h"

namespace road_network {

struct Node {
  Node(NodeID id, NodeName name) : id_(id), name_(std::move(name)) {}
  const NodeID id_;  // internal node ID
  const NodeName name_;
};

class Link {
 private:
  std::shared_ptr<Node> from_;
  std::shared_ptr<Node> to_;

  double capacity_{};
  double free_speed_{};
  double length_{};
  int lanes_{};
  int link_type_{};

  std::function<double(double)> link_performance_function_{};

 public:
  Link() = default;
  ~Link() = default;
  Link(NodePtr from, NodePtr to, double capacity, double free_speed,
       double length, int lanes, int link_type,
       LinkPerformanceFunction link_performance_function)
      : from_(std::move(from)),
        to_(std::move(to)),
        capacity_(capacity),
        free_speed_(free_speed),
        length_(length),
        lanes_(lanes),
        link_type_(link_type),
        link_performance_function_(std::move(link_performance_function)) {}

  // copy constructor
  Link(const Link& other) = delete;
  // copy assignment operator
  Link& operator=(const Link& other) = delete;
  // move constructor
  Link(Link&& other) noexcept;
  // move assignment operator
  Link& operator=(Link&& other) noexcept;

  // Travel time
  [[nodiscard]] double TravelTime(double flow) const {
    return link_performance_function_(flow);
  }
};

class Graph {
 public:
  // A new empty graph.
  // Currently, we only support creating a new graph with fixed number of nodes.
  // That is, the caller must specify the maximum number of nodes when creating
  // the graph.
  // TODO: support dynamic graph.
  explicit Graph(int n_nodes)
      : n_nodes_(n_nodes),
        adjacency_matrix_(Matrix<std::optional<Link>>(n_nodes, n_nodes)) {
    // resize the internal matrices
    link_flows_.resize(n_nodes, n_nodes);
    target_link_flows_.resize(n_nodes, n_nodes);
    link_costs_.resize(n_nodes, n_nodes);
    shortest_path_.Resize(n_nodes, n_nodes);
  }
  ~Graph() = default;

  // Currently, a Graph object is not copyable or movable.
  Graph() = delete;
  Graph(Graph& other) = delete;
  Graph& operator=(Graph& other) = delete;
  Graph(Graph&& other) noexcept = delete;
  Graph& operator=(Graph&& other) noexcept = delete;

  bool AddLink(const NodeName& from, const NodeName& to, double capacity,
               double free_speed, double length, int lanes, int link_type,
               LinkPerformanceFunction f);

  bool AddDemand(const NodeName& from, const NodeName& to, double demand);

  // Get Link object by node names.
  const std::optional<Link>& GetLink(const NodeName& from, const NodeName& to);

  // Get a Node pointer by node name.
  std::shared_ptr<Node> GetNode(const NodeName& name) {
    return node_id_map_[name].second;
  }

 public:
  // update the shortest path matrix for origin node r using Dijkstra's
  // algorithm
  // TODO: these functions should be private
  void UpdateSingleLinkFlow(const NodeName& from, const NodeName& to,
                            double flow);
  void UpdateLinkCosts();
  // DEBUG
  Eigen::MatrixXd& GetLinkFlows() { return link_flows_; }
  Eigen::MatrixXd& GetLinkCosts() { return link_costs_; }
  void PrintLinks();
  void PrintShortestPath();

  friend class Assignment;
  friend class MSA;

 private:
  // returns true if the node is successfully created or already exists.
  [[nodiscard]] bool MakeNode(const std::string& name);

  // returns true if the link is successfully created.
  bool AddLinkInternal(const NodePtr& from, const NodePtr& to, double capacity,
                       double free_speed, double length, int lanes,
                       int link_type, LinkPerformanceFunction f);

 public:
  void UpdateShortestPath(NodeID r);
  void UpdateShortestPathMany(const NodeSet& r_set);
  double ComputeAEC();

 private:
  // Each node is allowed to have a name of string type.
  // When a node is created, it is assigned an internal integer ID.
  std::unordered_map<NodeName, std::pair<NodeID, NodePtr>> node_id_map_{};
  std::unordered_map<NodeID, std::pair<NodeName, NodePtr>>
      node_id_reverse_map_{};
  NodeID node_id_counter_{};  // current max node ID + 1
  NodeID n_nodes_{};          // max number of nodes
  Matrix<std::optional<Link>> adjacency_matrix_{};
  // The demand matrix is represented as a map from origin node ID to a map from
  // destination node ID to demand.
  std::map<NodeID, std::map<NodeID, double>> demands_{};
  // Temporary variables for traffic assignment
  Eigen::MatrixXd link_flows_{};  // initialized to zero
  Eigen::MatrixXd target_link_flows_{};
  Eigen::MatrixXd link_costs_{};  // updated by UpdateLinkCosts()
  Matrix<std::pair<double, std::vector<NodeID>>> shortest_path_{};
};

}  // namespace road_network

#endif  // TRAFFIC_ASSIGNMENT_ROAD_NETWORK_GRAPH_H_
