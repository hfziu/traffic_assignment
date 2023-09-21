//
// Created by Xiaofei ZHAO on 29/8/2023.
//

#include "road_network/graph.h"

#include <string>
#include <utility>

namespace road_network {

Link::Link(NodePtr from, NodePtr to, double capacity, double length,
           double free_speed, int lanes, int link_type,
           LinkPerformanceFunction link_performance_function)
    : from_(std::move(from)),
      to_(std::move(to)),
      capacity_(capacity),
      length_(length),
      free_speed_(free_speed),
      lanes_(lanes),
      link_type_(link_type),
      link_performance_function_(std::move(link_performance_function)) {}

Link& Link::operator=(const Link& other) {
  if (this != &other) {
    from_ = other.from_;
    to_ = other.to_;
    capacity_ = other.capacity_;
    free_speed_ = other.free_speed_;
    length_ = other.length_;
    lanes_ = other.lanes_;
    link_type_ = other.link_type_;
    link_performance_function_ = other.link_performance_function_;
  }
  return *this;
}

Link::Link(Link&& other) noexcept
    : from_(std::move(other.from_)),
      to_(std::move(other.to_)),
      capacity_(other.capacity_),
      free_speed_(other.free_speed_),
      length_(other.length_),
      lanes_(other.lanes_),
      link_type_(other.link_type_),
      link_performance_function_(std::move(other.link_performance_function_)) {}

Link& Link::operator=(road_network::Link&& other) noexcept {
  if (this != &other) {
    from_ = std::move(other.from_);
    to_ = std::move(other.to_);
    capacity_ = other.capacity_;
    free_speed_ = other.free_speed_;
    length_ = other.length_;
    lanes_ = other.lanes_;
    link_type_ = other.link_type_;
    link_performance_function_ = std::move(other.link_performance_function_);
  }
  return *this;
}

Graph::Graph(int n_nodes)
    : n_nodes_(n_nodes),
      links_(n_nodes, n_nodes),
      shortest_path_(n_nodes, n_nodes),
      link_costs_(n_nodes, n_nodes, std::numeric_limits<double>::max()) {
  // resize the internal matrices
  link_flows_.resize(n_nodes, n_nodes);
  target_link_flows_.resize(n_nodes, n_nodes);
}

bool Graph::MakeNode(const NodeName& name) {
  if (node_id_counter_ > n_nodes_) {
    return false;
  }
  if (node_id_map_.find(name) == node_id_map_.end()) {
    node_id_map_[name].first = node_id_counter_;
    node_id_map_[name].second =
        std::make_shared<Node>(Node(node_id_counter_, name));
    node_id_reverse_map_[node_id_counter_].first = name;
    node_id_reverse_map_[node_id_counter_++].second = node_id_map_[name].second;
  }
  return true;
}

bool Graph::AddLink(const NodeName& from, const NodeName& to, double capacity,
                    double length, double free_speed, int lanes, int link_type,
                    LinkPerformanceFunction f) {
  if (MakeNode(from) && MakeNode(to)) {
    return AddLinkInternal(node_id_map_[from].second, node_id_map_[to].second,
                           capacity, length, free_speed, lanes, link_type,
                           std::move(f));
  } else {
    return false;
  }
}

bool Graph::AddLinkInternal(const NodePtr& from, const NodePtr& to,
                            double capacity, double length, double free_speed,
                            int lanes, int link_type,
                            LinkPerformanceFunction f) {
  links_(from->id_, to->id_) = {from,       to,    capacity,  length,
                                free_speed, lanes, link_type, std::move(f)};
  return true;
}

bool Graph::AddDemand(const NodeName& from, const NodeName& to, double demand) {
  if (MakeNode(from) && MakeNode(to)) {
    demands_[node_id_map_[from].first][node_id_map_[to].first] = demand;
    return true;
  } else {
    return false;
  }
}

void Graph::UpdateSingleLinkFlow(const NodeName& from, const NodeName& to,
                                 double flow) {
  link_flows_.coeffRef(node_id_map_[from].first, node_id_map_[to].first) = flow;
}

void Graph::UpdateLinkCosts() {
  for (auto& from_node : links_) {
    for (auto& link : from_node.second) {
      // if (node_id_reverse_map_[from_node.first].first.length() > 7 ||
      //     node_id_reverse_map_[link.first].first.length() > 7) {
      //   link_costs_(from_node.first, link.first) = 100000;  // a large number
      //   continue;
      // }
      link_costs_(from_node.first, link.first) = link.second.TravelTime(
          link_flows_.coeff(from_node.first, link.first));
    }
  }
}

void Graph::UpdateShortestPath(NodeID r) {
  // initialize the shortest distance array from node r
  double dist[node_id_counter_];
  std::fill(dist, dist + node_id_counter_, std::numeric_limits<double>::max());
  dist[r] = 0;                        // distance from r to r is 0
  shortest_path_(r, r).second = {r};  // initialize the path vector
  // finalized[i] is true if the shortest distance from r to i is finalized
  bool finalized[node_id_counter_];
  std::fill(finalized, finalized + node_id_counter_, false);

  // find the shortest path from r to all other nodes
  for (int i = 0; i < node_id_counter_ - 1; ++i) {
    // find the node with the minimum distance from r
    int u = -1;
    double min_dist = std::numeric_limits<double>::max();
    for (int j = 0; j < node_id_counter_; ++j) {
      if (!finalized[j] && dist[j] < min_dist) {
        u = j;
        min_dist = dist[j];
      }
    }
    if (u == -1) {
      break;
    }
    finalized[u] = true;
    shortest_path_(r, u).first = min_dist;
    // update the distance from r to all other nodes
    for (int v = 0; v < node_id_counter_; ++v) {
      if (!finalized[v] && dist[u] + link_costs_.at(u, v) < dist[v]) {
        dist[v] = dist[u] + link_costs_.at(u, v);
        shortest_path_(r, v).second = shortest_path_(r, u).second;
        shortest_path_(r, v).second.push_back(v);
        shortest_path_(r, v).first = dist[v];
      }
    }
  }
}

void Graph::UpdateShortestPathMany(const NodeSet& r_set) {
  for (auto& r : r_set) {
    UpdateShortestPath(r);
  }
}

double Graph::ComputeAEC() {
  double actual = 0;
  for (auto& from_node : links_) {
    for (auto& link : from_node.second) {
      auto travel_time =
          // node_id_reverse_map_[from_node.first].first.length() > 7 ||
          //         node_id_reverse_map_[link.first].first.length() > 7
          //     ? 100000 :
              link.second.TravelTime(
                    link_flows_.coeff(from_node.first, link.first));
      actual += travel_time * link_flows_.coeff(from_node.first, link.first);
    }
  }

  double shortest = 0;
  double sum_d = 0;
  for (auto& [r, s_map] : demands_) {
    for (auto& [s, d] : s_map) {
      shortest += shortest_path_(r, s).first * d;
      sum_d += d;
    }
  }
  return (actual - shortest) / sum_d;
}

// DEBUG
void Graph::PrintLinks() {
  for (auto& from_node : links_) {
    for (auto& link : from_node.second) {
      std::cout << "from: " << node_id_reverse_map_[from_node.first].first
                << ", to: " << node_id_reverse_map_[link.first].first
                << std::endl;
    }
  }
}

// DEBUG
void Graph::PrintShortestPath() {
  for (auto& from_node : shortest_path_) {
    for (auto& to_node : from_node.second) {
      if (!to_node.second.second.empty()) {
        std::cout << "from: " << node_id_reverse_map_[from_node.first].first
                  << ", to: " << node_id_reverse_map_[to_node.first].first
                  << ", distance: " << to_node.second.first << std::endl;
        std::cout << "path: ";
        for (auto& node : to_node.second.second) {
          std::cout << node_id_reverse_map_[node].first << " ";
        }
        std::cout << std::endl;
      }
    }
  }
}

}  // namespace road_network