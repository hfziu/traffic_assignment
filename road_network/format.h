//
// Created by Xiaofei ZHAO on 29/8/2023.
//

#ifndef TRAFFIC_ASSIGNMENT_ROAD_NETWORK_FORMAT_H_
#define TRAFFIC_ASSIGNMENT_ROAD_NETWORK_FORMAT_H_

#include <functional>
#include <set>

#include <Eigen/Sparse>

namespace road_network {

struct Node;

using NodeID = int;
using NodeSet = std::set<NodeID>;  // a set of node internal IDs
using NodeName = std::string_view;
using NodePtr = std::shared_ptr<Node>;
using LinkPerformanceFunction = std::function<double(double)>;

// Link flow matrix is represented by a sparse matrix, whose default value is 0.
using LinkFlowMatrix = Eigen::SparseMatrix<double>;

}  // namespace road_network

#endif  // TRAFFIC_ASSIGNMENT_ROAD_NETWORK_FORMAT_H_
