#include "SafeReachable.hpp"

#include <algorithm>

// num_step_bound starts as INT_MAX by default
//
// Note: origin is included, and if doing this multiple times in multiple direction pairs, may have duplicates.
void Preprocessing::get_safe_reachable_in_directions(const Graph &graph, std::vector<map_position> &output, const map_position origin, const Direction straight_direction, const Direction diagonal_direction, const int num_step_bound)
{
  assert (origin < graph.num_positions());

  map_position curr_pos = origin;
  int num_steps = 0;

  // Add points in straight direction until we hit an obstacle or reach the step bound
  while (!graph.is_obstacle(curr_pos) && num_steps < num_step_bound)
  {
    output.push_back(curr_pos);

    moving_direction(straight_direction, curr_pos, graph);
    num_steps++;
  }

  // If we hit an obstacle, then in the next recursion we can only go at most
  // one fewer steps, since we don't allow corner cutting.
  // This does not apply if we did not hit an obstacle
  if (graph.is_obstacle(curr_pos) && num_steps != 0)
    num_steps--;

  // Move in diagonal direction, and recursively call with appropriate bound
  // that ensures all points added are safe-reachable
  if (num_steps != 0)
  {
    map_position new_origin = origin;
    moving_direction(diagonal_direction, new_origin, graph);
    get_safe_reachable_in_directions(graph, output, new_origin, straight_direction, diagonal_direction, num_steps);
  }
}

// Note: origin is included
std::vector<map_position> Preprocessing::get_safe_reachable_in_all_directions(const Graph &graph, const map_position origin)
{
  std::vector<map_position> return_value;

  get_safe_reachable_in_directions(graph, return_value, origin, Dir_N, Dir_NW);
  get_safe_reachable_in_directions(graph, return_value, origin, Dir_N, Dir_NE);
  get_safe_reachable_in_directions(graph, return_value, origin, Dir_E, Dir_NE);
  get_safe_reachable_in_directions(graph, return_value, origin, Dir_E, Dir_SE);
  get_safe_reachable_in_directions(graph, return_value, origin, Dir_S, Dir_SE);
  get_safe_reachable_in_directions(graph, return_value, origin, Dir_S, Dir_SW);
  get_safe_reachable_in_directions(graph, return_value, origin, Dir_W, Dir_SW);
  get_safe_reachable_in_directions(graph, return_value, origin, Dir_W, Dir_NW);

  // May have duplicates, since the same point may be reachable for multiple
  // direction pairs, for example the origin is reachable for all direction
  // pairs. The other example is that points purely in the main_direction are
  // reachable regardless of the secondary direction.
  std::sort(return_value.begin(), return_value.end());
  const auto last = std::unique(return_value.begin(), return_value.end());
  return_value.erase(last, return_value.end());
  
  return return_value;
}

