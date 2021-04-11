#ifndef CORNERGRAPH_PROJECT__DIRECTIONS_HPP
#define CORNERGRAPH_PROJECT__DIRECTIONS_HPP

enum Direction {
  Dir_N  = 0,
  Dir_NE = 1,
  Dir_E  = 2,
  Dir_SE = 3,
  Dir_S  = 4,
  Dir_SW = 5,
  Dir_W  = 6,
  Dir_NW = 7,
  Dir_MIN = Dir_N,
  Dir_MAX = Dir_NW,
};

inline void moving_direction(unsigned int dir, map_position & pos, const Graph &graph);
inline exact_distance exact_distance_of_direction(Direction dir);

inline void moving_direction(unsigned int dir, map_position & pos, const Graph &graph)
{
  switch (dir) {
  case Dir_N:
  case Dir_NE:
  case Dir_NW:
    pos = graph.up(pos);
    break;
  case Dir_S:
  case Dir_SE:
  case Dir_SW:
    pos = graph.down(pos);
    break;
  default:
    break;
  }
  
  switch (dir) {
  case Dir_E:
  case Dir_NE:
  case Dir_SE:
    pos = graph.right(pos);
    break;
  case Dir_W:
  case Dir_NW:
  case Dir_SW:
    pos = graph.left(pos);
    break;
  default:
    break;
  }
}

inline exact_distance exact_distance_of_direction(Direction dir)
{
  switch (dir)
  {
  case Dir_N:
  case Dir_E:
  case Dir_S:
  case Dir_W:
    return exact_distance(1,0);
  case Dir_NE:
  case Dir_SE:
  case Dir_SW:
  case Dir_NW:
    return exact_distance(0,1);
  default:
    throw std::runtime_error("invalid direction");
  }
}

#endif
