#ifndef CORNERGRAPH_PROJECT__CORNER_VECTOR_SPLITTABLE_HPP
#define CORNERGRAPH_PROJECT__CORNER_VECTOR_SPLITTABLE_HPP

#include "../Graph/Graph.hpp"
#include "../Graph/MapPosition.hpp"
#include "CornerVector.hpp"
#include "RelevantPoints.hpp"

// All corners for which the divdirections can be split into two (not necessarily mutually exclusive) classes:
// Class 1: those which have no next corners when outgoing from this corner
// Class 2: those which when we are incoming from the opposite direction, the relevant directions are all in class 1
class CornerVectorSplittable
{
  private:
  std::vector<map_position> corners_splittable;

  public:
  void preprocess(const Graph &graph, const CornerVector &corner_vector, const RelevantPoints &relevant_points);
};

#endif
