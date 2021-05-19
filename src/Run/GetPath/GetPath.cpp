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

  //std::cout << _point_to_nearby_corner_indices_with_next[start].size() << " " << _point_to_nearby_corner_indices_with_next[goal].size() << " " << _point_to_nearby_corner_indices_with_next[start].size() * _point_to_nearby_corner_indices_with_next[goal].size() << std::endl;

  // Test going through each pair of nearby indices
  for (corner_index i : _point_to_nearby_corner_indices_with_next[start])
  {
    exact_distance i_dist = graph.octile_distance(start, _corners[i]);
    for (corner_index j : _point_to_nearby_corner_indices_with_next[goal])
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
