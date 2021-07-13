#ifndef CORNERGRAPH_PROJECT__RELEVANT_POINTS_HPP
#define CORNERGRAPH_PROJECT__RELEVANT_POINTS_HPP

#include "../Graph/Graph.hpp"
#include "../Graph/Directions.hpp"
#include "CornerVector.hpp"
#include "NearbyCorners.hpp"

class RelevantPoints
{
  private:
  std::vector<std::vector<std::vector<map_position>>> corner_to_divdirection_to_relevant_points;
  std::vector<std::vector<std::vector<corner_index>>> corner_to_divdirection_to_relevant_corners;

  void get_relevant_points(corner_index i, DivDirection divdirection, const Graph &graph);

  public:
  void preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners);
};

#endif
