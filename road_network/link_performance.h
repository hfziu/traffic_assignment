//
// Created by Xiaofei ZHAO on 29/8/2023.
//

#ifndef TRAFFIC_ASSIGNMENT_ROAD_NETWORK_LINK_PERFORMANCE_H_
#define TRAFFIC_ASSIGNMENT_ROAD_NETWORK_LINK_PERFORMANCE_H_

#include "road_network/format.h"

namespace road_network {

// Return a BPR function.
// t0: free flow travel time
// u: capacity
// alpha: BPR alpha
// beta: BPR beta
LinkPerformanceFunction BPR(double t0, double u, double alpha, double beta) {
  return [t0, u, alpha, beta](double x) -> double {
    return t0 * (1 + alpha * pow(x / u, beta));
  };
}

}  // namespace road_network

#endif  // TRAFFIC_ASSIGNMENT_ROAD_NETWORK_LINK_PERFORMANCE_H_
