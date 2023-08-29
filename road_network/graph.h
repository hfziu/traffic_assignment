//
// Created by Xiaofei ZHAO on 29/8/2023.
//

#ifndef TRAFFIC_ASSIGNMENT_ROAD_NETWORK_GRAPH_H_
#define TRAFFIC_ASSIGNMENT_ROAD_NETWORK_GRAPH_H_

#include <memory>
#include <string>
#include <utility>

#include "road_network/format.h"
#include "road_network/matrix.h"

namespace road_network {

struct Node {
  Node(int id, std::string name) : id_(id), name_(std::move(name)) {}
  const int id_;  // internal node ID
  const std::string name_;
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
  Link(std::shared_ptr<Node> from, std::shared_ptr<Node> to, double capacity,
       double free_speed, double length, int lanes, int link_type,
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
      : adjacency_matrix_(Matrix<Link>(n_nodes, n_nodes)) {}
  ~Graph() = default;

  // Currently, a Graph object is not copyable or movable.
  Graph() = delete;
  Graph(Graph& other) = delete;
  Graph& operator=(Graph& other) = delete;
  Graph(Graph&& other) noexcept = delete;
  Graph& operator=(Graph&& other) noexcept = delete;

  bool AddLink(const std::string& from, const std::string& to, double capacity,
               double free_speed, double length, int lanes, int link_type,
               LinkPerformanceFunction f);

  // Get Link object by node names.
  Link& GetLink(const std::string&& from, const std::string&& to);

  // Get Node internal ID by node name.
  int GetNodeId(const std::string& name) const {
    return node_id_map_.at(name).first;
  }

 private:
  bool AddLinkInternal(const std::shared_ptr<Node>& from,
                       const std::shared_ptr<Node>& to, double capacity,
                       double free_speed, double length, int lanes,
                       int link_type, LinkPerformanceFunction f) {
    adjacency_matrix_(from->id_, to->id_) =
        std::move(Link(from, to, capacity, free_speed, length, lanes, link_type,
                       std::move(f)));
    return true;
  }

 private:
  // Each node is allowed to have a name of string type.
  // When a node is created, it is assigned an internal integer ID.
  std::unordered_map<std::string, std::pair<int, std::shared_ptr<Node>>>
      node_id_map_{};
  int node_id_counter_{};  // max node ID + 1
  Matrix<Link> adjacency_matrix_{};
};

}  // namespace road_network

#endif  // TRAFFIC_ASSIGNMENT_ROAD_NETWORK_GRAPH_H_
