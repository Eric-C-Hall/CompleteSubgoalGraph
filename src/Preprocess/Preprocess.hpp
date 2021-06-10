#ifndef CORNERGRAPH_PROJECT__PREPROCESS_HPP
#define CORNERGRAPH_PROJECT__PREPROCESS_HPP

#include <vector>
#include <climits>

#include "../Graph/Graph.hpp"
#include "../Graph/XYLoc.hpp"
#include "../Graph/Directions.hpp"

typedef unsigned int corner_index;

void PreprocessMap(const Graph &graph);

class PreprocessingData {
  private:
  const Graph &graph;

  std::vector<map_position> _corners;
  std::vector<std::vector<corner_index>> _point_to_nearby_corner_indices;
  // A subset of the above vector, which only includes corner indices that are
  // relevent to some corner when entered from the direction of the origin point
  std::vector<std::vector<corner_index>> _point_to_nearby_corner_indices_with_next;

  std::vector<std::vector<exact_distance>> _pair_of_corner_indices_to_dist;
  // _pair_of_corner_indices_to_first_corner[i][j] is first corner from j to i
  std::vector<std::vector<corner_index>> _pair_of_corner_indices_to_first_corner;

  // Given a point, and a nearby corner with next, give me some bounds within
  // which all optimal destinations fall, when passing through these points.
  // first is the lower x and y bound, while second is the upper x and y bound
  std::vector<std::vector<std::pair<xyLoc, xyLoc>>> _corner_and_middirection_to_bounds;


  void _add_if_corner(map_position p);
  void _compute_corners();
  void _find_points_near_corner(corner_index i);
  void _find_nearby_corners();
  void _remove_useless_nearby_corners();
  void _remove_indirect_nearby_corners();

  void _compute_neighbouring_relevant_corners(std::vector<std::vector<std::vector<corner_index>>> &corner_and_direction_to_neighbouring_relevant_corners) const;
  void _find_corners_with_relevant_next_corners(std::vector<std::vector<std::vector<corner_index>>> &corner_and_direction_to_neighbouring_relevant_corners);

  void _find_optimal_distances_from_corner(corner_index i);
  void _find_optimal_first_corners_from_corner_to_corner(corner_index i, corner_index j);
  void _find_optimal_first_corners_from_corner(corner_index i);
  void _find_complete_corner_graph();

  void _push_corners_in_corner_graph();

  void _compute_safe_reachable_bounds(std::vector<std::pair<xyLoc, xyLoc>> &corner_to_bounds) const;
  void _find_geometric_container(corner_index i, MidDirection middirection, std::vector<std::pair<xyLoc, xyLoc>> &corner_to_bounds, const std::vector<std::vector<std::vector<corner_index>>> &corner_and_direction_to_neighbouring_relevant_corners);
  void _find_geometric_containers(std::vector<std::pair<xyLoc, xyLoc>> &corner_to_bounds, const std::vector<std::vector<std::vector<corner_index>>> &corner_and_direction_to_neighbouring_relevant_corners);

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

  bool get_path(map_position start, map_position goal, std::vector<xyLoc> &path) const;
  const Graph &get_graph() const {return graph;}

  std::vector<map_position> get_nearby_corners(map_position p) const;
  std::vector<map_position> get_nearby_corners_with_next(map_position p) const;

  inline const std::vector<map_position> & get_corners() const {return _corners;}
  inline const std::pair<xyLoc, xyLoc> & get_bounds(const corner_index i, const MidDirection middirection) const {return _corner_and_middirection_to_bounds[i][middirection];}
  inline const unsigned int get_num_nearby_corners_with_next(map_position p) const {return _point_to_nearby_corner_indices_with_next[p].size();}
  inline const map_position get_ith_nearby_corner_with_next(map_position p, int i) const {return _corners[_point_to_nearby_corner_indices_with_next[p][i]];}
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
