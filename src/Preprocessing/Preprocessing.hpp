#ifndef CORNERGRAPH_PROJECT__PREPROCESSING_HPP
#define CORNERGRAPH_PROJECT__PREPROCESSING_HPP

#include <vector>
#include <climits>

#include "../Graph/Graph.hpp"
#include "../Graph/XYLoc.hpp"
#include "../Graph/Directions.hpp"
#include "CornerVector.hpp"
#include "NearbyCorners.hpp"
#include "CompleteCornerGraph.hpp"

void PreprocessMap(const Graph &graph);

class PreprocessingData {
  private:
  const Graph &graph;

  CornerVector corner_vector;
  NearbyCorners nearby_corners;
  CompleteCornerGraph complete_corner_graph;

  void _save(std::ostream & stream) const;
  void _load(std::istream &stream);

  void _output_debug_stats() const;
  void _output_warnings() const;

  template<bool x_diff_greater_than_y_diff, bool test_valid>
  void _compute_diagonal_then_straight_path(xyLoc start, int16_t num_diagonal, int16_t num_straight, int16_t x_step, int16_t y_step, std::vector<xyLoc> &path) const;
  template<bool test_valid>
  void _compute_octile_path(xyLoc start, xyLoc end, std::vector<xyLoc> &path) const;

  bool _try_direct_path(map_position start, map_position goal) const;

  public:
  PreprocessingData(const Graph &graph);

  void preprocess();
  void save(const std::string &filename) const;
  void load(const std::string &filename);

  const Graph &get_graph() const {return graph;}
  const CornerVector &get_corner_vector() const {return corner_vector;}
  const NearbyCorners &get_nearby_corners() const {return nearby_corners;}
  const CompleteCornerGraph &get_complete_corner_graph() const {return complete_corner_graph;}  
};

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

#endif
