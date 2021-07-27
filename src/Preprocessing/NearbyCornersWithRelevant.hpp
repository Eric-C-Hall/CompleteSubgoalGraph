#ifndef CORNERGRAPH_PROJECT__NEARBY_CORNERS_WITH_RELEVANT_HPP
#define CORNERGRAPH_PROJECT__NEARBY_CORNERS_WITH_RELEVANT_HPP

#include "../Graph/Graph.hpp"
#include "CornerVector.hpp"
#include "NearbyCorners.hpp"
#include "RelevantPoints.hpp"

#include <vector>

class NearbyCornersWithRelevant
{
  private:
  std::vector<std::vector<corner_index>> point_to_nearby_corner_indices_with_relevant;

  public:
  void preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners, const RelevantPoints &relevant_points);
  void save(std::ostream &stream, const Graph &graph, const CornerVector &corner_vector) const;
  void load(std::istream &stream, const Graph &graph, const CornerVector &corner_vector);

  inline const std::vector<corner_index> &get_nearby_corner_indices_with_relevant(const map_position p) const {return point_to_nearby_corner_indices_with_relevant[p];}

  void print_nearby_with_relevant(map_position pos, Printer &printer, const Graph &graph, const CornerVector &corner_vector) const;
};

#endif
