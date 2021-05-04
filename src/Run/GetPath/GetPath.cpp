#include "GetPath.hpp"

#include <vector>

#include <cassert>

bool GetPath(const PreprocessingData &preprocessing_data, xyLoc s, xyLoc g, std::vector<xyLoc> &path)
{
  map_position start = preprocessing_data.get_graph().pos(s.x, s.y);
  map_position goal = preprocessing_data.get_graph().pos(g.x, g.y);

  return preprocessing_data.get_path(start, goal, path);
}

bool PreprocessingData::get_path(map_position start, map_position goal, std::vector<xyLoc> &path) const
{
  // Try some octile path
  path.push_back(graph.loc(start));
  _compute_octile_path<true>(graph.loc(start), graph.loc(goal), path);
  if (path.back() == graph.loc(goal))
  {
    return true;
  }
  path.clear();

  exact_distance shortest_distance = MAX_EXACT_DISTANCE;
  corner_index best_start_index = _corners.size();
  corner_index best_end_index = _corners.size();

  //std::cout << _point_to_nearby_corner_indices[start].size() << " " << _point_to_nearby_corner_indices[goal].size() << " " << _point_to_nearby_corner_indices[start].size() * _point_to_nearby_corner_indices[goal].size() << std::endl;

  for (corner_index i : _point_to_nearby_corner_indices[start])
  {
    exact_distance i_dist = graph.octile_distance(start, _corners[i]);
    for (corner_index j : _point_to_nearby_corner_indices[goal])
    {
      exact_distance j_dist = graph.octile_distance(goal, _corners[j]);
      exact_distance current_dist = i_dist + _pair_of_corner_indices_to_dist[i][j] + j_dist;
      if (current_dist < shortest_distance)
      {
        shortest_distance = current_dist;
        best_start_index = i;
        best_end_index = j;
      }
    }
  }

  xyLoc a = graph.loc(start);
  path.push_back(a);
  
  corner_index start_index = best_start_index;
  corner_index end_index = best_end_index;

  const auto &corner_index_to_next_corner = _pair_of_corner_indices_to_first_corner[end_index];

  xyLoc b = graph.loc(_corners[start_index]);

  _compute_octile_path<false>(a, b, path);
  while (start_index != end_index)
  {
    a = b;
    assert(start_index != _corners.size());
    assert(end_index != _corners.size());
    corner_index new_index = corner_index_to_next_corner[start_index];
    assert(new_index != start_index);
    start_index = new_index;
    b = graph.loc(_corners[start_index]);
    _compute_octile_path<false>(a, b, path);
  }
  _compute_octile_path<false>(b, graph.loc(goal), path);

  return true;
}

// Doesn't add start to path, but does add end.
template<bool x_diff_greater_than_y_diff, bool test_valid>
void PreprocessingData::_compute_diagonal_then_straight_path(xyLoc start, int16_t num_diagonal, int16_t num_straight, int16_t x_step, int16_t y_step, std::vector<xyLoc> &path) const
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

// Doesn't add start to path, but does add end.
template<bool test_valid>
void PreprocessingData::_compute_octile_path(xyLoc start, xyLoc end, std::vector<xyLoc> &path) const
{
  int16_t x_diff = std::abs(start.x - end.x);
  int16_t y_diff = std::abs(start.y - end.y);
  int16_t x_step = (start.x > end.x ? -1 : 1);
  int16_t y_step = (start.y > end.y ? -1 : 1);

  if (x_diff > y_diff)
    _compute_diagonal_then_straight_path<true, test_valid>(start, y_diff, x_diff - y_diff, x_step, y_step, path);
  else
    _compute_diagonal_then_straight_path<false, test_valid>(start, x_diff, y_diff - x_diff, x_step, y_step, path);
}

// eof
