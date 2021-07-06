#include "GetPath.hpp"

#include "DirectPath.hpp"

#include <vector>

#include <cassert>

bool GetPath(const PreprocessingData &preprocessing_data, xyLoc s, xyLoc g, std::vector<xyLoc> &path)
{
  const Graph &graph = preprocessing_data.get_graph();
  const CornerVector &corner_vector = preprocessing_data.get_corner_vector();

  map_position start = graph.pos(s.x, s.y);
  map_position goal = graph.pos(g.x, g.y);

  return Running::get_path(start, goal, path, graph, corner_vector);
}

template<bool try_octile, bool test_double, bool test_single, bool compute_path>
bool Running::get_path_partial_computation(map_position start, map_position goal, std::vector<xyLoc> &path, const Graph &graph, const CornerVector &corner_vector)
{
  assert (false);
}

bool Running::get_path_octile_step_only(map_position start, map_position goal, std::vector<xyLoc> &path, const Graph &graph, const CornerVector &corner_vector)
{
  return get_path_partial_computation<true,false,false,false>(start, goal, path, graph, corner_vector);
}

bool Running::get_path_up_to_double_step(map_position start, map_position goal, std::vector<xyLoc> &path, const Graph &graph, const CornerVector &corner_vector)
{
  return get_path_partial_computation<true,true,false,false>(start, goal, path, graph, corner_vector);
}

bool Running::get_path_up_to_single_step(map_position start, map_position goal, std::vector<xyLoc> &path, const Graph &graph, const CornerVector &corner_vector)
{
  return get_path_partial_computation<true,true,true,false>(start, goal, path, graph, corner_vector);
}

bool Running::get_path(map_position start, map_position goal, std::vector<xyLoc> &path, const Graph &graph, const CornerVector &corner_vector)
{
  return get_path_partial_computation<true,true,true,true>(start, goal, path, graph, corner_vector);
}

// eof
