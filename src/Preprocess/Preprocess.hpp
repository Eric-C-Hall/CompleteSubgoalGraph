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

  std::vector<std::vector<exact_distance>> _pair_of_corner_indices_to_dist;
  // _pair_of_corner_indices_to_first_corner[i][j] is first corner from j to i
  std::vector<std::vector<corner_index>> _pair_of_corner_indices_to_first_corner;


  void _add_if_corner(map_position p);
  void _compute_corners();
  template<Direction dir, Direction step_dir>
  void _find_points_near_corner_straight(map_position initial_pos, corner_index i, int num_step_bound = INT_MAX);
  void _find_points_near_corner(corner_index i);
  void _find_nearby_corners();
  void _remove_useless_nearby_corners();
  void _remove_indirect_nearby_corners();

  void _compute_neighbouring_relevant_corners(std::vector<std::vector<std::vector<corner_index>>> &corner_and_direction_to_neighbouring_relevant_corners);

  void _find_optimal_distances_from_corner(corner_index i);
  void _find_optimal_first_corners_from_corner_to_corner(corner_index i, corner_index j);
  void _find_optimal_first_corners_from_corner(corner_index i);
  void _find_complete_corner_graph();

  void _push_corners_in_corner_graph();

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

  inline const std::vector<map_position> & get_corners() const {return _corners;}
};

#endif
