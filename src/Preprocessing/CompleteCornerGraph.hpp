#ifndef CORNERGRAPH_PROJECT__COMPLETE_CORNER_GRAPH_HPP
#define CORNERGRAPH_PROJECT__COMPLETE_CORNER_GRAPH_HPP

#include <vector>

#include "../Graph/ExactDistance.hpp"
#include "CornerVector.hpp"

class CompleteCornerGraph
{
  private:
  // _pair_of_corner_indices_to_dist[i][j] only stored for j <= i
  std::vector<std::vector<exact_distance>> _pair_of_corner_indices_to_dist;
  // _pair_of_corner_indices_to_first_corner[i][j] is first corner from j to i
  std::vector<std::vector<corner_index>> _pair_of_corner_indices_to_first_corner;

  void _find_optimal_distances_from_corner(corner_index i, const CornerVector &corner_vector);
  void _find_optimal_first_corners_from_corner_to_corner(corner_index i, corner_index j, const CornerVector &corner_vector);
  void _find_optimal_first_corners_from_corner(corner_index i, const CornerVector &corner_vector);
  void _find_complete_corner_graph(const CornerVector &corner_vector);

  void _push_corners_in_corner_graph();

  public:
  void preprocess();
  void save(std::ostream &stream, const CornerVector &corner_vector) const;
  void load(std::istream &stream, const CornerVector &corner_vector);
};

#endif
