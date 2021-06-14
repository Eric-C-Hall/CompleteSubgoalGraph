#ifndef CORNERGRAPH_PROJECT__ISLANDS_HPP
#define CORNERGRAPH_PROJECT__ISLANDS_HPP

#include <vector>

#include "Graph.hpp"

class Islands {
  private:
  const Graph &graph;
  std::vector<int> island_index;
  std::vector<std::pair<xyLoc, xyLoc>> island_bounds;

  void create_island_from_point(map_position p, int curr_island_index);

  public:
  Islands(const Graph &input_graph);
  inline int get_island_index(map_position p) const {return island_index[p];}
  inline int get_island_index(xyLoc loc) const {return get_island_index(graph.pos(loc));}
  inline const std::pair<xyLoc, xyLoc> & get_island_bounds(int island_index) const {return island_bounds[island_index];}
  inline int get_num_islands() const {return island_bounds.size();}
};

#endif
