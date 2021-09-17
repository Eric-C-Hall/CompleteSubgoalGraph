#ifndef CORNERGRAPH_PROJECT__NEARBY_CORNERS_WITH_NEXT_HPP
#define CORNERGRAPH_PROJECT__NEARBY_CORNERS_WITH_NEXT_HPP

#include "../Graph/Graph.hpp"
#include "CornerVector.hpp"
#include "NearbyCorners.hpp"
#include "RelevantPoints.hpp"

#include <vector>

class NearbyCornersWithNext
{
  private:
  std::vector<std::vector<corner_index>> point_to_nearby_corner_indices_with_next;

  public:
  void preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners, const RelevantPoints &relevant_points);
  void remove_collar(const Graph &graph);
  void save(std::ostream &stream, const Graph &graph, const CornerVector &corner_vector) const;
  void load(std::istream &stream, const Graph &graph, const CornerVector &corner_vector);

  inline const std::vector<corner_index> &get_nearby_corner_indices_with_next(const map_position p) const {return point_to_nearby_corner_indices_with_next[p];}

  void print_nearby_with_next(map_position pos, Printer &printer, const Graph &graph, const CornerVector &corner_vector) const;
};

#endif
