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

bool Graph::AddLink(const std::string& from, const std::string& to,
                    double capacity, double free_speed, double length,
                    int lanes, int link_type, LinkPerformanceFunction f) {
  if (node_id_map_.find(from) == node_id_map_.end()) {
    node_id_map_[from].first = node_id_counter_++;
    node_id_map_[from].second =
        std::make_shared<Node>(Node(node_id_map_[from].first, from));
  }
  if (node_id_map_.find(to) == node_id_map_.end()) {
    node_id_map_[to].first = node_id_counter_++;
    node_id_map_[to].second =
        std::make_shared<Node>(Node(node_id_map_[to].first, to));
  }
  return AddLinkInternal(node_id_map_[from].second, node_id_map_[to].second,
                         capacity, free_speed, length, lanes, link_type, f);
}

Link& Graph::GetLink(const std::string&& from, const std::string&& to)
{
    return adjacency_matrix_(node_id_map_[from].first, node_id_map_[to].first);
  }

}  // namespace road_network