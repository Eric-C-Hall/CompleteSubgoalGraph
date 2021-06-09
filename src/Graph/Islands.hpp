#ifndef CORNERGRAPH_PROJECT__ISLANDS_HPP
#define CORNERGRAPH_PROJECT__ISLANDS_HPP

#include <vector>

#include "Graph.hpp"

class Islands {
  private:
  const Graph &graph;
  std::vector<int> island_index;

  public:
  Islands(const Graph &input_graph);
  inline int get_island_index(map_position p) const {return island_index[p];}
  inline int get_island_index(xyLoc loc) const {return get_island_index(graph.pos(loc));}
};

#endif
