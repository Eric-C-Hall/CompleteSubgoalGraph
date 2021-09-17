#include "GetPath.hpp"

#include "DirectPath.hpp"
#include "../Graph/Reachable.hpp"

#include <vector>

#include <cassert>

template<bool try_octile, bool test_double, bool test_single, bool compute_path>
bool Running::get_path_partial_computation(map_position start, map_position goal, std::vector<xyLoc> &path, const PreprocessingData &preprocessing_data, const Graph &graph)
{
  const CornerVector &corner_vector = preprocessing_data.get_corner_vector();
  const NearbyCornersWithRelevant &nearby_corners_with_relevant = preprocessing_data.get_nearby_corners_with_relevant();
  const NearbyCornersWithNext &nearby_corners_with_next = preprocessing_data.get_nearby_corners_with_next();
  const CompleteCornerGraph &complete_corner_graph = preprocessing_data.get_complete_corner_graph();
  const GeometricContainersIncoming &geometric_containers_incoming = preprocessing_data.get_geometric_containers_incoming();

  path.clear();

  // TODO: we have now converted goal and start from xyLoc to map_position and back again. This is not ideal.
  // On the other hand, the time taken is probably insignificant.
  // NOTE: for some reason when I tried taking xyLoc as input and converting to map_position instead of other way around, that seemed to cause it to run slower
  xyLoc start_loc = graph.loc(start);
  xyLoc goal_loc = graph.loc(goal);

  if (try_octile)
  {
    // Try some octile path
    path.push_back(start_loc);
    Running::compute_octile_path<true>(start_loc, goal_loc, path, graph);
    if (path.back() == goal_loc)
    {
      return true;
    }
    path.clear();
  }

  exact_distance shortest_distance = MAX_EXACT_DISTANCE;
  corner_index best_start_index = corner_vector.size();
  corner_index best_end_index = corner_vector.size();

  if (test_double)
  {
    // Find goal corner indices for which the start is in the correct bounding box
    std::vector<corner_index> goal_test_corner_indices;
    for (const corner_index i : nearby_corners_with_next.get_nearby_corner_indices_with_next(goal))
    {
      const DivDirection divdirection = get_divdirection_between_points(goal_loc, graph.loc(corner_vector.get_corner(i)));
      const Bounds &bounds = geometric_containers_incoming.get_bounds(i, divdirection);
      
      if (bounds.contains(start_loc))
        goal_test_corner_indices.push_back(i);
    }

    // Try all start corner indices for which the goal is in the correct bounding box, and update best_start_index and best_end_index if necessary
    for (corner_index i : nearby_corners_with_next.get_nearby_corner_indices_with_next(start))
    {
      const map_position ci = corner_vector.get_corner(i);
      const xyLoc ci_loc = graph.loc(ci);
      const DivDirection divdirection = get_divdirection_between_points(start_loc, ci_loc);
      const Bounds &bounds = geometric_containers_incoming.get_bounds(i, divdirection);

      if (!bounds.contains(goal_loc))
        continue;

      const exact_distance i_dist = Reachable::octile_distance(graph, start, ci);
      for (corner_index j : goal_test_corner_indices)
      {
        const map_position cj = corner_vector.get_corner(j);
        const exact_distance j_dist = Reachable::octile_distance(graph, goal, cj);
        const exact_distance current_dist = i_dist + (i == j ? ZERO_EXACT_DISTANCE : complete_corner_graph.get_exact_distance_between_corner_indices(i,j)) + j_dist;
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
    const auto &start_nearby = nearby_corners_with_relevant.get_nearby_corner_indices_with_relevant(start);
    const auto &goal_nearby = nearby_corners_with_relevant.get_nearby_corner_indices_with_relevant(goal);

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
          const exact_distance current_dist = Reachable::octile_distance(graph, start, corner_vector.get_corner(*s_iter)) + Reachable::octile_distance(graph, goal, corner_vector.get_corner(*s_iter));
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

    const auto &corner_index_to_next_corner = complete_corner_graph.get_corner_index_to_first_corner(end_index);

    xyLoc b = graph.loc(corner_vector.get_corner(start_index));

    Running::compute_octile_path<false>(a, b, path, graph);
    while (start_index != end_index)
    {
      a = b;
      assert(start_index != corner_vector.size());
      assert(end_index != corner_vector.size());
      corner_index new_index = corner_index_to_next_corner[start_index];
      assert(new_index != start_index);
      start_index = new_index;
      b = graph.loc(corner_vector.get_corner(start_index));
      Running::compute_octile_path<false>(a, b, path, graph);
    }
    Running::compute_octile_path<false>(b, graph.loc(goal), path, graph);

    return true;
  }
  else
  {
    path.push_back(start_loc);
    path.push_back(graph.loc(corner_vector.get_corner(best_start_index)));
    path.push_back(graph.loc(corner_vector.get_corner(best_end_index)));
    path.push_back(goal_loc);
    return false;
  }

}

bool Running::get_path_octile_step_only(map_position start, map_position goal, std::vector<xyLoc> &path, const PreprocessingData &preprocessing_data, const Graph &graph)
{
  return get_path_partial_computation<true,false,false,false>(start, goal, path, preprocessing_data, graph);
}

bool Running::get_path_up_to_double_step(map_position start, map_position goal, std::vector<xyLoc> &path, const PreprocessingData &preprocessing_data, const Graph &graph)
{
  return get_path_partial_computation<true,true,false,false>(start, goal, path, preprocessing_data, graph);
}

bool Running::get_path_up_to_single_step(map_position start, map_position goal, std::vector<xyLoc> &path, const PreprocessingData &preprocessing_data, const Graph &graph)
{
  return get_path_partial_computation<true,true,true,false>(start, goal, path, preprocessing_data, graph);
}

bool Running::get_path(map_position start, map_position goal, std::vector<xyLoc> &path, const PreprocessingData &preprocessing_data, const Graph &graph)
{
  return get_path_partial_computation<true,true,true,true>(start, goal, path, preprocessing_data, graph);
}

// eof
