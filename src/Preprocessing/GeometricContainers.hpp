#ifndef CORNERGRAPH_PROJECT__GEOMETRIC_CONTAINERS_HPP
#define CORNERGRAPH_PROJECT__GEOMETRIC_CONTAINERS_HPP

#include <vector>
#include <algorithm>
#include <deque>

#include "../Graph/Graph.hpp"
#include "CompleteCornerGraph.hpp"
#include "CornerVector.hpp"
#include "RelevantPoints.hpp"

class Bound
{
  private:
  xyLoc upper_bound;
  xyLoc lower_bound;

  public:
  Bound(xyLoc input_upper_bound, xyLoc input_lower_bound) : upper_bound(input_upper_bound), lower_bound(input_lower_bound) {}
  inline Bound combine(const Bound &other) const;
  void print(Printer &printer, const Highlight &highlight);
};

class GeometricContainersOutgoing
{
  private:
  std::vector<std::vector<Bound>> corner_to_outgoing_direction_to_immediate_bound;
  std::vector<std::vector<Bound>> corner_to_outgoing_direction_to_bound;

  Bound search_relevant(const corner_index i, const DivDirection outgoing_divdirection, const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points);

  void preprocess_immediate(const Graph &graph, const CornerVector &corner_vector, const RelevantPoints &relevant_points);
  void preprocess_overall(const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points);

  inline const Bound &get_immediate_bound(const corner_index i, const DivDirection d) const {return corner_to_outgoing_direction_to_immediate_bound[i][d];}

  public:
  void preprocess(const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points);
  void print_bound(const corner_index i, const DivDirection dir, Printer &printer);
  void print_immediate_bound(const corner_index i, const DivDirection dir, Printer &printer);
  void print_all_bounds(const Graph &graph, const CornerVector &corner_vector);
};

class GeometricContainersIncoming
{
  private:
  std::vector<std::vector<Bound>> corner_to_incoming_direction_to_bound;

  public:
};

inline Bound Bound::combine(const Bound &other) const
{
  xyLoc new_upper_bound = xyLoc(std::max(upper_bound.x, other.upper_bound.x), std::max(upper_bound.y, other.upper_bound.y));
  xyLoc new_lower_bound = xyLoc(std::min(lower_bound.x, other.lower_bound.x), std::min(lower_bound.y, other.lower_bound.y));
  return Bound(new_upper_bound, new_lower_bound);
}

#endif
