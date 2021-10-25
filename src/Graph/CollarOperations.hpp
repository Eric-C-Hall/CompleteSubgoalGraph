#ifndef CORNERGRAPH_PROJECT__COLLAR_OPERATIONS_HPP
#define CORNERGRAPH_PROJECT__COLLAR_OPERATIONS_HPP

#include "Graph.hpp"

#include <vector>
#include <set>

namespace CollarOperations
{
  // Given a vector from map_position to some type of data, remove the collar
  template<class T>
  void remove_collar_map(std::vector<T> &vec, const Graph &graph)
  {
    int old_width = graph.get_width();
    int old_height = graph.get_height();
    int new_width = old_width-2;
    int new_height = old_height-2;

    std::vector<T> new_vec;
    new_vec.resize(new_width * new_height);

    for (int y = 0; y < new_height; y++)
    {
      for (int x = 0; x < new_width; x++)
      {
        new_vec[y * new_width + x] = vec[graph.pos(x+1,y+1)];
      }
    }

    vec.swap(new_vec);
  }

  // Take a map_position and return the map_position without the collar
  inline map_position remove_collar_pos(map_position p, const Graph &graph)
  {
    xyLoc l = graph.loc(p);
    l.x--;
    l.y--;
    return l.y * (graph.get_width()-2) + l.x;
  }

  // Given a vector of map_positions, remove the collar
  inline void remove_collar_each(std::vector<map_position> &vec, const Graph &graph)
  {
    for (map_position &p : vec)
      p = remove_collar_pos(p, graph);
  }

  inline void remove_collar_each(std::set<map_position> &poses, const Graph &graph)
  {
    std::set<map_position> new_set;
    for (map_position p : poses)
    {
        new_set.insert(remove_collar_pos(p,graph));
    }
    poses.swap(new_set);
  }
}
#endif
