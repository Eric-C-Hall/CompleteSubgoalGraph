#ifndef CORNERGRAPH_PROJECT__DIRECT_PATH_HPP
#define CORNERGRAPH_PROJECT__DIRECT_PATH_HPP

#include <cstdint>
#include <vector>

#include "../../Graph/XYLoc.hpp"
#include "../../Graph/Graph.hpp"

namespace Running
{
  template<bool test_valid>
  void compute_octile_path(xyLoc start, xyLoc end, std::vector<xyLoc> &path, const Graph &graph);

  // This should not be used by the user, it should only be used by compute_octile_path
  template<bool x_diff_greater_than_y_diff, bool test_valid>
  static void compute_diagonal_then_straight_path(xyLoc start, int16_t num_diagonal, int16_t num_straight, int16_t x_step, int16_t y_step, std::vector<xyLoc> &path, const Graph &graph);
}

// Doesn't add start to path, but does add end.
template<bool test_valid>
void Running::compute_octile_path(xyLoc start, xyLoc end, std::vector<xyLoc> &path, const Graph &graph)
{
  int16_t x_diff = std::abs(start.x - end.x);
  int16_t y_diff = std::abs(start.y - end.y);
  int16_t x_step = (start.x > end.x ? -1 : 1);
  int16_t y_step = (start.y > end.y ? -1 : 1);

  if (x_diff > y_diff)
    compute_diagonal_then_straight_path<true, test_valid>(start, y_diff, x_diff - y_diff, x_step, y_step, path, graph);
  else
    compute_diagonal_then_straight_path<false, test_valid>(start, x_diff, y_diff - x_diff, x_step, y_step, path, graph);
}

// Doesn't add start to path, but does add end.
template<bool x_diff_greater_than_y_diff, bool test_valid>
static void Running::compute_diagonal_then_straight_path(xyLoc start, int16_t num_diagonal, int16_t num_straight, int16_t x_step, int16_t y_step, std::vector<xyLoc> &path, const Graph &graph)
{
  while(num_diagonal > 0)
  {
    start.x += x_step;
    start.y += y_step;
    if (test_valid)
      if (graph.is_obstacle(graph.pos(start.x, start.y)))
        return;
    path.push_back(start);
    num_diagonal--;
  }
  while(num_straight > 0)
  {
    if (x_diff_greater_than_y_diff)
      start.x += x_step;
    else
      start.y += y_step;
    if (test_valid)
      if (graph.is_obstacle(graph.pos(start.x, start.y)))
        return;
    path.push_back(start);
    num_straight--;
  }
}

#endif
