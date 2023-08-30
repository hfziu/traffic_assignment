//
// Created by Xiaofei ZHAO on 29/8/2023.
//

#include "road_network/graph.h"

namespace road_network {

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
                    double free_speed, double length, int lanes, int link_type,
                    LinkPerformanceFunction f) {
  if (MakeNode(from) && MakeNode(to)) {
    return AddLinkInternal(node_id_map_[from].second, node_id_map_[to].second,
                           capacity, free_speed, length, lanes, link_type, f);
  } else {
    return false;
  }
}

bool Graph::AddLinkInternal(const NodePtr& from, const NodePtr& to,
                            double capacity, double free_speed, double length,
                            int lanes, int link_type,
                            LinkPerformanceFunction f) {
  adjacency_matrix_(from->id_, to->id_) = {
      from, to, capacity, free_speed, length, lanes, link_type, std::move(f)};
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

const std::optional<Link>& Graph::GetLink(const NodeName& from,
                                          const NodeName& to) {
  return adjacency_matrix_(node_id_map_[from].first, node_id_map_[to].first);
}

void Graph::UpdateSingleLinkFlow(const NodeName& from, const NodeName& to,
                                 double flow) {
  link_flows_(node_id_map_[from].first, node_id_map_[to].first) = flow;
}

void Graph::UpdateLinkCosts() {
  // initialize the link cost matrix
  for (int i = 0; i < n_nodes_; ++i) {
    for (int j = 0; j < n_nodes_; ++j) {
      if (i == j) {
        link_costs_(i, j) = 0;
      } else if (adjacency_matrix_(i, j) == std::nullopt) {
        // no direct link from i to j
        link_costs_(i, j) = std::numeric_limits<double>::max();
      } else {
        link_costs_(i, j) =
            adjacency_matrix_(i, j)->TravelTime(link_flows_(i, j));
      }
    }
  }
}

void Graph::UpdateShortestPath(NodeID r) {
  // initialize the shortest distance array from node r
  double dist[n_nodes_];
  std::fill(dist, dist + n_nodes_, std::numeric_limits<double>::max());
  dist[r] = 0;                        // distance from r to r is 0
  shortest_path_(r, r).second = {r};  // initialize the path vector
  // finalized[i] is true if the shortest distance from r to i is finalized
  bool finalized[n_nodes_];
  std::fill(finalized, finalized + n_nodes_, false);

  // find the shortest path from r to all other nodes
  for (int i = 0; i < n_nodes_ - 1; ++i) {
    // find the node with the minimum distance from r
    int u = -1;
    double min_dist = std::numeric_limits<double>::max();
    for (int j = 0; j < n_nodes_; ++j) {
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
    for (int v = 0; v < n_nodes_; ++v) {
      if (!finalized[v] && dist[u] + link_costs_(u, v) < dist[v]) {
        dist[v] = dist[u] + link_costs_(u, v);
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
  for (int i = 0; i < n_nodes_; ++i) {
    for (int j = 0; j < n_nodes_; ++j) {
      if (adjacency_matrix_(i, j).has_value()) {
        actual += adjacency_matrix_(i, j)->TravelTime(link_flows_(i, j)) *
               link_flows_(i, j);
      }
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
  for (int i = 0; i < n_nodes_; ++i) {
    for (int j = 0; j < n_nodes_; ++j) {
      if (adjacency_matrix_(i, j).has_value()) {
        std::cout << "from: " << node_id_reverse_map_[i].first
                  << ", to: " << node_id_reverse_map_[j].first << std::endl;
      }
    }
  }
}

// DEBUG
void Graph::PrintShortestPath() {
  for (int i = 0; i < n_nodes_; ++i) {
    for (int j = 0; j < n_nodes_; ++j) {
      if (!shortest_path_(i, j).second.empty()) {
        std::cout << "from: " << node_id_reverse_map_[i].first
                  << ", to: " << node_id_reverse_map_[j].first
                  << ", distance: " << shortest_path_(i, j).first << std::endl;
        std::cout << "path: ";
        for (auto& node : shortest_path_(i, j).second) {
          std::cout << node_id_reverse_map_[node].first << " ";
        }
        std::cout << std::endl;
      }
    }
  }
}

}  // namespace road_network