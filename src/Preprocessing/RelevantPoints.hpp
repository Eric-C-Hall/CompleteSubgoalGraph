#ifndef CORNERGRAPH_PROJECT__RELEVANT_POINTS_HPP
#define CORNERGRAPH_PROJECT__RELEVANT_POINTS_HPP

#include "../Graph/Graph.hpp"
#include "../Graph/Directions.hpp"
#include "CornerVector.hpp"
#include "NearbyCorners.hpp"

class RelevantPoints
{
  private:
  std::vector<std::vector<std::set<DivDirection>>> corner_to_incoming_divdirection_to_relevant_outgoing_divdirections;
  std::vector<std::vector<std::vector<map_position>>> corner_to_outgoing_divdirection_to_relevant_points;
  std::vector<std::vector<std::vector<corner_index>>> corner_to_outgoing_divdirection_to_relevant_corners;

  void update_outgoing_divdirections(std::vector<std::set<DivDirection>> & divdirection_to_divdirections, const DivDirection corner_divdirection, const DivDirection relative_divdirection, const std::vector<DivDirection> &added_relative_divdirections);
  void compute_relevant_divdirections(const corner_index i, const DivDirection corner_divdirection, const Graph &graph, const CornerVector &corner_vector);
  //void get_relevant_points(corner_index i, DivDirection divdirection, const Graph &graph);
  //void get_relevant_corners(corner_index i, DivDirection divdirection, const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners);

  public:
  void preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners);
};

#endif
