#ifndef CORNERGRAPH_PROJECT__ENTRY_H
#define CORNERGRAPH_PROJECT__ENTRY_H

#include "../../Preprocess/Preprocess.hpp"
#include "../../Graph/XYLoc.hpp"

inline bool GetPath(const PreprocessingData &preprocessing_data, xyLoc s, xyLoc g, std::vector<xyLoc> &path);

// GetPath.cpp also implements some functions defined in Preprocess.hpp


inline bool GetPath(const PreprocessingData &preprocessing_data, xyLoc s, xyLoc g, std::vector<xyLoc> &path)
{
  return preprocessing_data.get_path(s, g, path);
}

inline bool PreprocessingData::get_path(xyLoc start_loc, xyLoc goal_loc, std::vector<xyLoc> &path) const
{
  return get_path_partial_computation<true, true, true, true>(start_loc, goal_loc, path);
}

template<bool try_octile, bool test_double, bool test_single, bool compute_path>
bool PreprocessingData::get_path_partial_computation(xyLoc start_loc, xyLoc goal_loc, std::vector<xyLoc> &path) const
{
  map_position start = graph.pos(start_loc);
  map_position goal = graph.pos(goal_loc);

  if (try_octile && try_octile_path(start_loc, goal_loc, path))
    return true;

  exact_distance shortest_distance = MAX_EXACT_DISTANCE;
  corner_index best_start_index = _corners.size();
  corner_index best_end_index = _corners.size();

  if (test_double)
    test_double_paths(start_loc, goal_loc, start, goal, shortest_distance, best_start_index, best_end_index);
  if (test_single)
    test_single_paths(start, goal, shortest_distance, best_start_index, best_end_index);
  if (compute_path)
    compute_best_path_found(start_loc, goal_loc, best_start_index, best_end_index, path);
  else
  {
    path.push_back(start_loc);
    path.push_back(graph.loc(_corners[best_start_index]));
    path.push_back(graph.loc(_corners[best_end_index]));
    path.push_back(goal_loc);
  }

  return true;
}

#endif
