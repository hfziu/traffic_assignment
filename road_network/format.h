//
// Created by Xiaofei ZHAO on 29/8/2023.
//

#ifndef TRAFFIC_ASSIGNMENT_ROAD_NETWORK_FORMAT_H_
#define TRAFFIC_ASSIGNMENT_ROAD_NETWORK_FORMAT_H_

#include <functional>

namespace road_network {

struct Node;

using NodeID = int;
using NodeName = std::string;
using NodePtr = std::shared_ptr<Node>;
using LinkPerformanceFunction = std::function<double(double)>;

}  // namespace road_network

#endif  // TRAFFIC_ASSIGNMENT_ROAD_NETWORK_FORMAT_H_
