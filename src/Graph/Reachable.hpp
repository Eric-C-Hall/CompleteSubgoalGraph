#ifndef CORNERGRAPH_PROJECT__SAFE_REACHABLE_HPP
#define CORNERGRAPH_PROJECT__SAFE_REACHABLE_HPP

#include <vector>

#include "../Graph/MapPosition.hpp"
#include "../Graph/Directions.hpp"
#include "../Graph/Graph.hpp"

namespace Reachable
{
  // Safe Reachable
  bool is_safe_reachable(const Graph &graph, const map_position a, const map_position b);

  void get_safe_reachable_in_directions(const Graph &graph, std::vector<map_position> &output, const map_position origin, const Direction straight_direction, const Direction diagonal_direction, const int num_step_bound = INT_MAX);
  std::vector<map_position> get_safe_reachable_in_all_directions(const Graph &graph, const map_position origin);
  std::vector<map_position> get_safe_reachable_in_all_directions_heavy_assert(const Graph &graph, const map_position origin);

  // Octile Reachable
  inline exact_distance octile_distance(const Graph &graph, const map_position a, const map_position b);


  // Helper functions
  bool can_make_move(const Graph &graph, const map_position p, const Direction dir);
}

// ------------------
// Implementations
// ------------------

inline exact_distance Reachable::octile_distance(const Graph &graph, const map_position a, const map_position b)
{
  unsigned int x_a = graph.x(a);
  unsigned int x_b = graph.x(b);
  unsigned int y_a = graph.y(a);
  unsigned int y_b = graph.y(b);

  unsigned int abs_x_diff = (x_a > x_b ? x_a - x_b : x_b - x_a);
  unsigned int abs_y_diff = (y_a > y_b ? y_a - y_b : y_b - y_a);

  int num_diagonal = std::min(abs_x_diff, abs_y_diff);
  int num_straight = std::max(abs_x_diff, abs_y_diff) - num_diagonal;
  return exact_distance(num_straight, num_diagonal);
}

#endif
