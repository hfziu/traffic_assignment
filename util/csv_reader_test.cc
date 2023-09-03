//
// Created by Xiaofei ZHAO on 3/9/2023.
//

#include "util/csv_reader.h"

#include <gtest/gtest.h>

namespace road_network::util {

TEST(CSVReaderTest, SplitString) {
  std::string_view str = "1,2,3,4,5";
  auto output = SplitString(str, ',');
  ASSERT_EQ(output.size(), 5);
  ASSERT_EQ(output[0], "1");
  ASSERT_EQ(output[1], "2");
  ASSERT_EQ(output[2], "3");
  ASSERT_EQ(output[3], "4");
  ASSERT_EQ(output[4], "5");
}

TEST(CSVReaderTest, ReadSanFranciscoLink) {
  // TODO: read number of nodes from info file
  auto graph = Graph(4986);
  auto str = GetFileContents(
      "../data/01_san_francisco/base_info/san_francisco_link.csv");
  auto lines = SplitLines(str, true);  // skip header
  for (auto&& line : lines) {
    ASSERT_TRUE(AddLinkFromLine(graph, line));
  }
}

}  // namespace road_network::util