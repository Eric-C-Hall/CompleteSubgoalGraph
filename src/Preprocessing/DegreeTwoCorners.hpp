#ifndef CORNERGRAPH_PROJECT__DEGREE_TWO_CORNER_VECTOR_HPP
#define CORNERGRAPH_PROJECT__DEGREE_TWO_CORNER_VECTOR_HPP

#include "../Graph/MapPosition.hpp"
#include "../Graph/Graph.hpp"
#include "CornerVector.hpp"
#include "NearbyCorners.hpp"
#include "CompleteCornerGraph.hpp"

class DegreeTwoCorners
{
  private:
  std::vector<bool> is_degree_two;

  bool path_through_index_optimal(corner_index a, corner_index b, corner_index i, const CompleteCornerGraph &complete_corner_graph) const;
  bool no_path_through_other_indices(corner_index a, corner_index b, corner_index i, const CornerVector &corner_vector, const NearbyCorners &nearby_corners, const CompleteCornerGraph &complete_corner_graph) const;
  bool compute_is_degree_one(corner_index i, const CornerVector &corner_vector, const NearbyCorners &nearby_corners, const CompleteCornerGraph &complete_corner_graph) const;

  public:
  void preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners, const CompleteCornerGraph &complete_corner_graph);

  void print(Printer &printer, const Graph &graph, const CornerVector &corner_vector) const;
};

#endif
