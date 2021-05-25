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
  // TODO: we have now converted goal and start from xyLoc to map_position and back again. This is not ideal.
  // On the other hand, the time taken is probably insignificant.
  xyLoc start_loc = graph.loc(start);
  xyLoc goal_loc = graph.loc(goal);

  // Try some octile path
  path.push_back(start_loc);
  _compute_octile_path<true>(start_loc, goal_loc, path);
  if (path.back() == goal_loc)
  {
    return true;
  }
  path.clear();

  exact_distance shortest_distance = MAX_EXACT_DISTANCE;
  corner_index best_start_index = _corners.size();
  corner_index best_end_index = _corners.size();

  //std::cout << _point_to_nearby_corner_indices_with_next[start].size() << " " << _point_to_nearby_corner_indices_with_next[goal].size() << " " << _point_to_nearby_corner_indices_with_next[start].size() * _point_to_nearby_corner_indices_with_next[goal].size() << std::endl;

  // Find goal corner indices for which the start is in the correct bounding box
  std::vector<corner_index> goal_test_corner_indices;
  const auto &goal_nearby_corner_indices = _point_to_nearby_corner_indices_with_next[goal];
  for (unsigned int i = 0; i < goal_nearby_corner_indices.size(); i++)
  {
    const std::pair<xyLoc, xyLoc> bounds = _point_and_corner_to_bounds[goal][i];
    
    if (bounds.first.x <= start_loc.x && bounds.first.y <= start_loc.y && bounds.second.x >= start_loc.x && bounds.second.y >= start_loc.y)
    {
      goal_test_corner_indices.push_back(goal_nearby_corner_indices[i]);
    }
  }

  // Test going through each pair of nearby indices
  const auto &start_test_corner_indices = _point_to_nearby_corner_indices_with_next[start];
  for (unsigned int index = 0; index < start_test_corner_indices.size(); index++)
  {
    // Check if goal is in the correct bounding box for this corner index
    const std::pair<xyLoc, xyLoc> bounds = _point_and_corner_to_bounds[start][index];
    if (bounds.first.x > start_loc.x || bounds.first.y > start_loc.y || bounds.second.x < start_loc.x || bounds.second.y < start_loc.y)
    {
      continue;
    }

    const int i = start_test_corner_indices[index];

    exact_distance i_dist = graph.octile_distance(start, _corners[i]);
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

  // Compute best path found
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

// eof
