//
// Created by Xiaofei ZHAO on 3/9/2023.
//

#include <filesystem>
#include <fstream>

#include "road_network/graph.h"
#include "road_network/link_performance.h"

namespace fs = std::filesystem;

namespace road_network::util {

[[nodiscard]] std::string GetFileContents(const fs::path& filePath) {
  std::ifstream inFile{filePath, std::ios::in | std::ios::binary};
  if (!inFile)
    throw std::runtime_error("Cannot open " + filePath.filename().string());

  std::string str(static_cast<size_t>(fs::file_size(filePath)), 0);

  inFile.read(str.data(), str.size());
  if (!inFile)
    throw std::runtime_error("Could not read the full contents from " +
                             filePath.filename().string());

  return str;
}

[[nodiscard]] std::vector<std::string_view> SplitString(std::string_view str,
                                                        char delim) {
  std::vector<std::string_view> output;

  const auto last = str.end();
  for (auto first = str.begin(), second = str.begin();
       second != last && first != last; first = std::next(second)) {
    second = std::find(first, last, delim);

    // we might get empty string views here, but that's ok in the case of CSV
    // reader
    // output.emplace_back(str.substr(std::distance(str.begin(), first),
    // std::distance(first, second)));
    output.emplace_back(&*first, std::distance(first, second));

    if (second == last) break;
  }

  return output;
}

[[nodiscard]] std::vector<std::string_view> SplitLines(
    std::string_view str, bool skip_header = false) {
  auto lines = SplitString(str, '\n');
  if (skip_header) {
    lines.erase(lines.begin());
  }

  if (!lines.empty() && lines[0].back() == '\r') {  // Windows CR conversion
    for (auto&& line : lines) {
      if (line.back() == '\r') line.remove_suffix(1);
    }
  }

  return lines;
}

bool AddLinkFromLine(Graph& g, std::string_view line) {
  auto fields = SplitString(line, ',');
  if (fields.size() != 8) {
    // Link_ID,From_Node_ID,To_Node_ID,Capacity,Length,Free_Speed,Lanes,Link_Type
    throw std::runtime_error("Invalid line length for link file, should be 8");
  }
  auto from = fields[1];
  auto to = fields[2];
  auto capacity = std::stod(std::string(fields[3]));
  auto length = std::stod(std::string(fields[4])) / 1000; //meters to km
  auto free_speed = std::stod(std::string(fields[5])) * 1.60934; // miles/h to km/h
  auto lanes = std::stoi(std::string(fields[6]));
  auto link_type = std::stoi(std::string(fields[7]));
  // TODO: read BPR parameters from file. 0.5 and 2 are for San Francisco only.
  g.AddLink(from, to, capacity, length, free_speed, lanes, link_type,
            BPR(length / free_speed, capacity, 0.5, 2));
  return true;
}

bool AddDemandFromLine(Graph& g, std::string_view line) {
  auto fields = SplitString(line, ',');
  if (fields.size() != 3) {
    // From_Node_ID,To_Node_ID,Demand
    throw std::runtime_error(
        "Invalid line length for demand file, should be 3");
  }
  auto from = fields[0];
  auto to = fields[1];
  auto demand = std::stod(std::string(fields[2])) * 0.6;
  g.AddDemand(from, to, demand);
  return true;
}

// write link flow to file
void WriteLinkFlow(Graph& g, const fs::path& file_path) {
  std::ofstream file(file_path);
  // create file if not exist
  if (!file) {
    std::ofstream(file_path).close();
    file.open(file_path);
  }
  file << "From_Node_ID,To_Node_ID,Link_Flow,Link_Cost" << "\n";
  for (auto& from_node : g.GetLinks()) {
    for (auto& to_node : from_node.second) {
      file << g.GetNodeName(from_node.first) << "," << g.GetNodeName(to_node.first)
           << "," << g.GetLinkFlows().coeff(from_node.first, to_node.first)
           << "," << g.GetLinks().at(from_node.first, to_node.first).TravelTime(
               g.GetLinkFlows().coeff(from_node.first, to_node.first)) * 60
           << "\n";
    }
  }
}

}  // namespace road_network::util
