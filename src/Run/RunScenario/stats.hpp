#ifndef CORNERGRAPH_PROJECT__STATS_HPP
#define CORNERGRAPH_PROJECT__STATS_HPP

#include <vector>

#include "../../Graph/XYLoc.hpp"
#include "../../Graph/Graph.hpp"

constexpr bool PRINT_DEBUG_PATH_IF_FAILS_TO_VALIDATE = true;

struct stats {
  std::vector<double> times;
  std::vector<xyLoc> path;
  std::vector<int> lengths;

  double GetTotalTime();
  double GetMaxTimestep();
  double Get20MoveTime();
  double GetPathLength();
  bool ValidatePath(int width, int height, const Graph &graph);

  void debug_print_path(int width, int height, const std::vector<bool>&mapData);
};

#endif
