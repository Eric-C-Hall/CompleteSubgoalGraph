#ifndef CORNERGRAPH_PROJECT__GEOMETRIC_CONTAINERS_HPP
#define CORNERGRAPH_PROJECT__GEOMETRIC_CONTAINERS_HPP

#include <vector>
#include <algorithm>
#include <deque>

#include "../Graph/Graph.hpp"
#include "CompleteCornerGraph.hpp"
#include "CornerVector.hpp"
#include "RelevantPoints.hpp"

class Bounds
{
  private:
  xyLoc upper_bound;
  xyLoc lower_bound;

  public:
  Bounds(xyLoc input_upper_bound, xyLoc input_lower_bound) : upper_bound(input_upper_bound), lower_bound(input_lower_bound) {}
  inline Bounds combine(const Bounds &other) const;
  void print(Printer &printer, const Highlight &highlight) const;

  void save(std::ostream &stream) const;
  void load(std::istream &stream);

  bool contains(const xyLoc &l) const {return upper_bound.x > l.x && upper_bound.y > l.y && lower_bound.x < l.x && lower_bound.y < l.y;}
};

class GeometricContainersOutgoing
{
  private:
  std::vector<std::vector<Bounds>> corner_to_outgoing_direction_to_immediate_bounds;
  std::vector<std::vector<Bounds>> corner_to_outgoing_direction_to_bounds;

  Bounds search_relevant(const corner_index i, const DivDirection outgoing_divdirection, const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points);

  void preprocess_immediate(const Graph &graph, const CornerVector &corner_vector, const RelevantPoints &relevant_points);
  void preprocess_overall(const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points);

  inline const Bounds &get_immediate_bounds(const corner_index i, const DivDirection d) const {return corner_to_outgoing_direction_to_immediate_bounds[i][d];}

  public:
  void preprocess(const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points);
  void print_bounds(const corner_index i, const DivDirection dir, Printer &printer) const;
  void print_immediate_bounds(const corner_index i, const DivDirection dir, Printer &printer) const;
  void print_all_bounds(const Graph &graph, const CornerVector &corner_vector) const;
  const Bounds &get_bounds(const corner_index i, const DivDirection dir) const  {return corner_to_outgoing_direction_to_bounds[i][dir];}
};

class GeometricContainersIncoming
{
  private:
  std::vector<std::vector<Bounds>> corner_to_incoming_direction_to_bounds;

  public:
  void convert_from(const GeometricContainersOutgoing geometric_containers_outgoing, const Graph &graph, const CornerVector &corner_vector, const RelevantPoints &relevant_points);

  void save(std::ostream &stream) const;
  void load(std::istream &stream, const CornerVector &corner_vector);

  const Bounds &get_bounds(const corner_index i, const DivDirection dir) const  {return corner_to_incoming_direction_to_bounds[i][dir];}

  void print_bounds(const corner_index i, const DivDirection dir, Printer &printer) const;
};

inline Bounds Bounds::combine(const Bounds &other) const
{
  xyLoc new_upper_bound = xyLoc(std::max(upper_bound.x, other.upper_bound.x), std::max(upper_bound.y, other.upper_bound.y));
  xyLoc new_lower_bound = xyLoc(std::min(lower_bound.x, other.lower_bound.x), std::min(lower_bound.y, other.lower_bound.y));
  return Bounds(new_upper_bound, new_lower_bound);
}

#endif
