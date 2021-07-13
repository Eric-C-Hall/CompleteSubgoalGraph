#ifndef CORNERGRAPH_PROJECT__DIRECT_PATH_HPP
#define CORNERGRAPH_PROJECT__DIRECT_PATH_HPP

#include <cstdint>
#include <vector>

#include "../../Graph/XYLoc.hpp"
#include "../../Graph/Graph.hpp"
#include "../../Visualise/Printer.hpp"

namespace Running
{
  template<bool test_valid>
  inline void compute_octile_path(xyLoc start, xyLoc end, std::vector<xyLoc> &path, const Graph &graph);

  void print_octile_path(xyLoc start, xyLoc end, Printer &printer, const Graph &graph);

}

// ------------------------------------
// Implementation
// ------------------------------------

namespace RunningHelper
{
  template<bool x_diff_greater_than_y_diff, bool test_valid>
  inline void compute_diagonal_then_straight_path(xyLoc start, int16_t num_diagonal, int16_t num_straight, int16_t x_step, int16_t y_step, std::vector<xyLoc> &path, const Graph &graph);
}

// Doesn't add start to path, but does add end.
template<bool test_valid>
inline void Running::compute_octile_path(xyLoc start, xyLoc end, std::vector<xyLoc> &path, const Graph &graph)
{
  int16_t x_diff = std::abs(start.x - end.x);
  int16_t y_diff = std::abs(start.y - end.y);
  int16_t x_step = (start.x > end.x ? -1 : 1);
  int16_t y_step = (start.y > end.y ? -1 : 1);

  if (x_diff > y_diff)
    RunningHelper::compute_diagonal_then_straight_path<true, test_valid>(start, y_diff, x_diff - y_diff, x_step, y_step, path, graph);
  else
    RunningHelper::compute_diagonal_then_straight_path<false, test_valid>(start, x_diff, y_diff - x_diff, x_step, y_step, path, graph);
}

// Doesn't add start to path, but does add end.
template<bool x_diff_greater_than_y_diff, bool test_valid>
inline void RunningHelper::compute_diagonal_then_straight_path(xyLoc start, int16_t num_diagonal, int16_t num_straight, int16_t x_step, int16_t y_step, std::vector<xyLoc> &path, const Graph &graph)
{
  while(num_diagonal > 0)
  {
    if (test_valid)
    {
      if (graph.is_obstacle(graph.pos(start.x, start.y + y_step)))
        return;

      start.x += x_step;
      if (graph.is_obstacle(graph.pos(start.x, start.y)))
        return;

      start.y += y_step;
      if (graph.is_obstacle(graph.pos(start.x, start.y)))
        return;
    }
    else
    {
      start.x += x_step;
      start.y += y_step;
    }
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
