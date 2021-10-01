#ifndef CORNERGRAPH_PROJECT__NEARBY_CORNERS_HPP
#define CORNERGRAPH_PROJECT__NEARBY_CORNERS_HPP

#include <vector>

#include "CornerVector.hpp"
#include "../Graph/Graph.hpp"

class NearbyCorners
{
  private:
  std::vector<std::vector<corner_index>> point_to_nearby_corner_indices;

  void find_points_near_corner(corner_index i, const Graph &graph, const CornerVector &corner_vector);
  void find_nearby_corners();
  public:
  void preprocess(const Graph &graph, const CornerVector &corner_vector);
  void save(std::ostream &stream, const Graph &graph, const CornerVector &corner_vector) const;
  void load(std::istream &stream, const Graph &graph, const CornerVector &corner_vector);

  inline const std::vector<corner_index> & get_nearby_corner_indices(map_position p) const {return point_to_nearby_corner_indices[p];}

  void print_nearby(map_position pos, Printer &printer, const Graph &graph, const CornerVector &corner_vector) const;
  void print(corner_index which_index, Printer &printer, const Graph &graph, const CornerVector &corner_vector) const;
  void print(corner_index which_index, const Graph &graph, const CornerVector &corner_vector) const;
  void print_all(const Graph &graph, const CornerVector &corner_vector) const;

  void print_stats(const Graph &graph) const;
};

#endif
