#ifndef CORNERGRAPH_PROJECT__COMPLETE_CORNER_GRAPH_HPP
#define CORNERGRAPH_PROJECT__COMPLETE_CORNER_GRAPH_HPP

#include <vector>

#include "../Graph/ExactDistance.hpp"
#include "CornerVector.hpp"
#include "NearbyCorners.hpp"
#include "../Visualise/Printer.hpp"

class CompleteCornerGraph
{
  private:
  // pair_of_corner_indices_to_dist[i][j] only stored for j < i
  std::vector<std::vector<exact_distance>> pair_of_corner_indices_to_dist;
  // pair_of_corner_indices_to_first_corner[i][j] is first corner from j to i, undefined for i == j
  std::vector<std::vector<corner_index>> pair_of_corner_indices_to_first_corner;

  void find_optimal_distances_from_corner(corner_index i, const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners);
  void find_optimal_first_corners_from_corner_to_corner(corner_index i, corner_index j, const CornerVector &corner_vector, const NearbyCorners &nearby_corners);
  void find_optimal_first_corners_from_corner(corner_index i, const CornerVector &corner_vector, const NearbyCorners &nearby_corners);

  void push_corners_in_corner_graph();

  public:
  inline exact_distance get_exact_distance_between_corner_indices(corner_index i, corner_index j) const;
  inline corner_index get_first_corner(corner_index i, corner_index j) const {return pair_of_corner_indices_to_first_corner[j][i];}
  inline const std::vector<corner_index> &get_corner_index_to_first_corner(corner_index i) const {return pair_of_corner_indices_to_first_corner[i];}

  void preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners);
  void save(std::ostream &stream, const Graph &graph, const CornerVector &corner_vector) const;
  void load(std::istream &stream, const Graph &graph, const CornerVector &corner_vector);

  void print_first(corner_index i, corner_index j, Printer &printer, const Graph &graph, const CornerVector &corner_vector) const;
  void print_first_and_dist(const corner_index i, const corner_index j, const Graph &graph, const CornerVector &corner_vector) const;
  void print_all_first_and_dist(const Graph &graph, const CornerVector &corner_vector) const;
};

// -------------------
// Implementation
// -------------------

inline exact_distance CompleteCornerGraph::get_exact_distance_between_corner_indices(corner_index i, corner_index j) const
{
  assert (i != j);

  if (i < j)
    std::swap(i,j);
  
  return pair_of_corner_indices_to_dist[i][j];
}

#endif
