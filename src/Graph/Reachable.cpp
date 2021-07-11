#include "Reachable.hpp"

#include <algorithm>

bool Reachable::can_make_move(const Graph &graph, const map_position p, const Direction dir)
{
  if (!is_cardinal_direction(dir))
  {
    map_position cut_a = graph.step_in_direction(p, get_45_degrees_clockwise(dir));
    map_position cut_b = graph.step_in_direction(p, get_45_degrees_anticlockwise(dir));
    if (graph.is_obstacle(cut_a) || graph.is_obstacle(cut_b))
      return false;
  }

  map_position new_pos = graph.step_in_direction(p, dir);
  if (graph.is_obstacle(new_pos))
  {
    return false;
  }

  return true;
}

std::vector<map_position> get_parallelogram_positions(const map_position start, const Direction straight_dir, const Direction diagonal_dir, const exact_distance num_straight_and_num_diagonal, const Graph &graph)
{
  std::vector<map_position> return_value;

  const int num_straight = num_straight_and_num_diagonal.num_straight;
  const int num_diagonal = num_straight_and_num_diagonal.num_diagonal;

  // Iterate over each position in the straight direction from start
  map_position base_pos = start;
  for (int i = 0; i <= num_straight; i++)
  {
    // Iterate over each position in the diagonal direction from the current position
    map_position curr_pos = base_pos;
    for (int j = 0; j <= num_diagonal; j++)
    {
      return_value.push_back(curr_pos);
      moving_direction(diagonal_dir, curr_pos, graph);
    }
    moving_direction(straight_dir, base_pos, graph);
  }

  return return_value;
}

// num_straight_and_num_diagonal should be the distance between the corners of the parallelogram
bool check_parallelogram_clear(const map_position start, const Direction straight_dir, const Direction diagonal_dir, const exact_distance num_straight_and_num_diagonal, const Graph &graph)
{
  for (map_position p : get_parallelogram_positions(start, straight_dir, diagonal_dir, num_straight_and_num_diagonal, graph))
  {
    if (graph.is_obstacle(p))
      return false;
  }
  return true;
}

std::vector<map_position> get_diagonal_first_cut_positions(const map_position start, Direction straight_dir, const Direction diagonal_dir, const exact_distance num_straight_and_num_diagonal, const Graph &graph)
{
  std::vector<map_position> return_value;

  // In the special case where we only need to move in the diagonal direction,
  // we can't trust that the input straight_dir is correct.
  // We should instead manually choose the straight_dir, making sure that when
  // the opposite diagonal direction is chosen, the opposite straight direction
  // is also chosen.
  if (num_straight_and_num_diagonal.num_straight == 0)
    straight_dir = get_45_degrees_clockwise(diagonal_dir);

  // Get first position to add
  map_position add_position = start;
  moving_direction(diagonal_dir, add_position, graph);
  moving_direction(get_opposite_direction(straight_dir), add_position, graph);

  // Add the relevant positions
  for (int i = 0; i < num_straight_and_num_diagonal.num_diagonal; i++)
  {
    return_value.push_back(add_position);
    moving_direction(diagonal_dir, add_position, graph);
  }

  return return_value;
}

bool check_diagonal_first_cut_positions_clear(const map_position start, const Direction straight_dir, const Direction diagonal_dir, const exact_distance num_straight_and_num_diagonal, const Graph &graph)
{
  for (map_position p : get_diagonal_first_cut_positions(start, straight_dir, diagonal_dir, num_straight_and_num_diagonal, graph))
    if (graph.is_obstacle(p))
      return false;
  return true;
}

bool Reachable::is_safe_reachable(const Graph &graph, const map_position a, const map_position b)
{
  if (a == b)
    return true;

  const xyLoc a_loc = graph.loc(a);
  const xyLoc b_loc = graph.loc(b);
  const Direction straight_dir = get_straight_direction_between_points(a_loc,b_loc);
  const Direction diagonal_dir = get_diagonal_direction_between_points(a_loc,b_loc);
  const exact_distance dist = octile_distance(graph,a,b);

  // Ensure all points directly on paths between a and b are clear
  if (!check_parallelogram_clear(a, straight_dir, diagonal_dir, dist, graph))
    return false;

  // Ensure that for diagonal-first or diagonal-last paths, the points that would be cut by corner-cutting are clear.
  if (!check_diagonal_first_cut_positions_clear(a, straight_dir, diagonal_dir, dist, graph))
    return false;
  if (!check_diagonal_first_cut_positions_clear(b, get_opposite_direction(straight_dir), get_opposite_direction(diagonal_dir), dist, graph))
    return false;

  return true;
}

// num_step_bound starts as INT_MAX by default
//
// Note: origin is included, and if doing this multiple times in multiple direction pairs, may have duplicates.
void Reachable::get_safe_reachable_in_directions(const Graph &graph, std::vector<map_position> &output, const map_position origin, const Direction straight_direction, const Direction diagonal_direction, const int num_step_bound)
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

  // If the first position was an obstacle, then no points are safe reachable from it.
  // Note: this may occur during a recursive call, even if an obstacle was not initially the chosen origin
  if (num_steps == 0)
    return;

  // If we hit an obstacle, then in the next recursion we can only go at most
  // one fewer steps, since we don't allow corner cutting.
  // This does not apply if we did not hit an obstacle.
  if (graph.is_obstacle(curr_pos))
    num_steps--;

  // Check that moving in diagonal direction does not require corner cutting.
  // If so, then move in this direction, and recursively call with appropriate
  // bound on the number of steps that ensures all points added are
  // safe-reachable
  map_position new_origin = graph.step_in_direction(origin, diagonal_direction);
  const map_position cut_location_a = graph.step_in_direction(new_origin, get_opposite_direction(straight_direction));
  const map_position cut_location_b = graph.step_in_direction(origin, straight_direction);
  if (graph.is_obstacle(cut_location_a) || graph.is_obstacle(cut_location_b))
    return;
  get_safe_reachable_in_directions(graph, output, new_origin, straight_direction, diagonal_direction, num_steps);
}

// Note: origin is included
std::vector<map_position> Reachable::get_safe_reachable_in_all_directions(const Graph &graph, const map_position origin)
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

std::vector<map_position> Reachable::get_safe_reachable_in_all_directions_heavy_assert(const Graph &graph, const map_position origin)
{
  #ifndef NDEBUG
  std::vector<map_position> return_value = get_safe_reachable_in_all_directions(graph, origin);

  std::vector<bool> is_safe_reachable_vec(graph.num_positions(), false);

  for (map_position m : return_value)
    is_safe_reachable_vec[m] = true;

  for (map_position m = 0; m < graph.num_positions(); m++)
  {
    if (is_safe_reachable_vec[m] != is_safe_reachable(graph, origin, m))
    {
      // These definitions are copy/pasted from is_safe_reachable
      const map_position a = origin;
      const map_position b = m;
      const xyLoc a_loc = graph.loc(a);
      const xyLoc b_loc = graph.loc(b);
      const Direction straight_dir = get_straight_direction_between_points(a_loc,b_loc);
      const Direction diagonal_dir = get_diagonal_direction_between_points(a_loc,b_loc);
      const exact_distance dist = octile_distance(graph,a,b);

      // Print debug data
      Printer printer;
      graph.print(printer);
      for (map_position p : get_parallelogram_positions(a, straight_dir, diagonal_dir, dist, graph))
        printer.add_highlight(Highlight(4), graph.loc(p));
      for (map_position p : get_diagonal_first_cut_positions(a, straight_dir, diagonal_dir, dist, graph))
        printer.add_highlight(Highlight(5), graph.loc(p));
      for (map_position p : get_diagonal_first_cut_positions(b, get_opposite_direction(straight_dir), get_opposite_direction(diagonal_dir), dist, graph))
        printer.add_highlight(Highlight(6), graph.loc(p));
      printer.add_highlight(Highlight(2), graph.loc(m));
      printer.add_highlight(Highlight(3), graph.loc(origin));
      printer.print();
      std::cout << "m: " << m << " a.k.a. " << graph.loc(m) << std::endl;
      std::cout << "origin: " << origin << " a.k.a. " << graph.loc(origin) << std::endl;
      std::cout << "vec: " << is_safe_reachable_vec[m] << std::endl;
      std::cout << "cal: " << is_safe_reachable(graph, origin, m) << std::endl;
      std::cout << "par: " << check_parallelogram_clear(a, straight_dir, diagonal_dir, dist, graph) << std::endl;
    }

    assert(is_safe_reachable_vec[m] == is_safe_reachable(graph, origin, m));
  }

  return return_value;
  #else
  return get_safe_reachable_in_all_directions(graph, origin);
  #endif
}

