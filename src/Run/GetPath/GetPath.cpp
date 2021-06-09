#include "GetPath.hpp"

#include <vector>

#include <cassert>

bool PreprocessingData::try_octile_path(xyLoc start_loc, xyLoc goal_loc, std::vector<xyLoc> &path) const
{
  // Try some octile path
  path.push_back(start_loc);
  _compute_octile_path<true>(start_loc, goal_loc, path);
  if (path.back() == goal_loc)
  {
    return true;
  }
  path.clear();
  return false;
}

void PreprocessingData::test_double_paths(xyLoc start_loc, xyLoc goal_loc, map_position start, map_position goal, exact_distance &shortest_distance, corner_index &best_start_index, corner_index &best_end_index) const
{
  // Find goal corner indices for which the start is in the correct bounding box
  std::vector<corner_index> goal_test_corner_indices;
  const auto &goal_nearby_corner_indices = _point_to_nearby_corner_indices_with_next[goal];
  for (const corner_index i : goal_nearby_corner_indices)
  {
    const MidDirection middirection = get_middirection_between_points(goal_loc, graph.loc(_corners[i]));
    const std::pair<xyLoc, xyLoc> bounds = _corner_and_middirection_to_bounds[i][middirection];
    
    if (graph.is_point_in_bounds(start_loc, bounds))
    {
      goal_test_corner_indices.push_back(i);
    }
  }

  // Test going through each pair of nearby indices with relevant next corners
  for (corner_index i : _point_to_nearby_corner_indices_with_next[start])
  {
    map_position ci = _corners[i];

    // Check if goal is in the correct bounding box for this corner index
    const MidDirection middirection = get_middirection_between_points(start_loc, graph.loc(ci));
    const std::pair<xyLoc, xyLoc> bounds = _corner_and_middirection_to_bounds[i][middirection];
    if (!graph.is_point_in_bounds(goal_loc, bounds))
    {
      continue;
    }

    // Try each choice of goal index given this start index, compare to best distance so far
    exact_distance i_dist = graph.octile_distance(start, ci);
    for (corner_index j : goal_test_corner_indices)
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
}

void PreprocessingData::test_single_paths(map_position start, map_position goal, exact_distance &shortest_distance, corner_index &best_start_index, corner_index &best_end_index) const
{
  // Test going through single nearby index possibly with no relevant next corner
  const auto &start_nearby = _point_to_nearby_corner_indices[start];
  const auto &goal_nearby = _point_to_nearby_corner_indices[goal];

  auto s_iter = start_nearby.begin();
  auto g_iter = goal_nearby.begin();

  while (true)
  {
    if (*s_iter < *g_iter)
    {
      s_iter++;
      if (s_iter == start_nearby.end())
        break;
    }
    else
    {
      if (*s_iter == *g_iter)
      {
        const exact_distance current_dist = graph.octile_distance(start, _corners[*s_iter]) + graph.octile_distance(goal, _corners[*s_iter]);
        if (current_dist < shortest_distance)
        {
          shortest_distance = current_dist;
          best_start_index = best_end_index = *s_iter;
        }
        s_iter++;
        if (s_iter == start_nearby.end())
          break;
        g_iter++;
        if (g_iter == goal_nearby.end())
          break;
      }
      else
      {
        g_iter++;
        if (g_iter == goal_nearby.end())
          break;
      }
    }
  }
}

void PreprocessingData::compute_best_path_found(xyLoc start_loc, xyLoc goal_loc, corner_index best_start_index, corner_index best_end_index, std::vector<xyLoc> &path) const
{
  // Compute best path found
  xyLoc a = start_loc;
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
  _compute_octile_path<false>(b, goal_loc, path);
}

// eof
