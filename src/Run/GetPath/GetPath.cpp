#include "GetPath.hpp"

#include <vector>

#include <cassert>

bool GetPath(const PreprocessingData &preprocessing_data, xyLoc s, xyLoc g, std::vector<xyLoc> &path)
{
  map_position start = preprocessing_data.get_graph().pos(s.x, s.y);
  map_position goal = preprocessing_data.get_graph().pos(g.x, g.y);

  return preprocessing_data.get_path(start, goal, path);
}

template<bool try_octile, bool test_double, bool test_single, bool compute_path>
bool PreprocessingData::get_path_partial_computation(map_position start, map_position goal, std::vector<xyLoc> &path) const
{
  // TODO: we have now converted goal and start from xyLoc to map_position and back again. This is not ideal.
  // On the other hand, the time taken is probably insignificant.
  // NOTE: for some reason when I tried taking xyLoc as input and converting to map_position instead of other way around, that seemed to cause it to run slower
  xyLoc start_loc = graph.loc(start);
  xyLoc goal_loc = graph.loc(goal);

  if (try_octile)
  {
    // Try some octile path
    path.push_back(start_loc);
    _compute_octile_path<true>(start_loc, goal_loc, path);
    if (path.back() == goal_loc)
    {
      return true;
    }
    path.clear();
  }

  exact_distance shortest_distance = MAX_EXACT_DISTANCE;
  corner_index best_start_index = _corners.size();
  corner_index best_end_index = _corners.size();

  //std::cout << _point_to_nearby_corner_indices_with_next[start].size() << " " << _point_to_nearby_corner_indices_with_next[goal].size() << " " << _point_to_nearby_corner_indices_with_next[start].size() * _point_to_nearby_corner_indices_with_next[goal].size() << std::endl;

  if (test_double)
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

  if (test_single)
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

  if (compute_path)
  {
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
  else
  {
    path.push_back(start_loc);
    path.push_back(graph.loc(_corners[best_start_index]));
    path.push_back(graph.loc(_corners[best_end_index]));
    path.push_back(goal_loc);
    return false;
  }
}

bool PreprocessingData::get_path(map_position start, map_position goal, std::vector<xyLoc> &path) const
{
  return get_path_partial_computation<true,true,true,true>(start, goal, path);
}

// eof
