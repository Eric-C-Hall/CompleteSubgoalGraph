#ifndef CORNERGRAPH_PROJECT__DIRECTIONS_HPP
#define CORNERGRAPH_PROJECT__DIRECTIONS_HPP

#include "XYLoc.hpp"

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

inline std::vector<Direction> get_cardinal_directions();

inline Direction get_45_degrees_clockwise(Direction dir);
inline Direction get_45_degrees_anticlockwise(Direction dir);

inline bool in_direction_from_point(xyLoc a, xyLoc b, Direction dir);
inline bool within_45_degrees_clockwise_from_point(xyLoc a, xyLoc b, Direction dir);
inline bool within_45_degrees_anticlockwise_from_point(xyLoc a, xyLoc b, Direction dir);

inline void moving_direction(unsigned int dir, map_position & pos, const Graph &graph);

inline exact_distance exact_distance_of_direction(Direction dir);

// --------------------------------------------------------

inline std::vector<Direction> get_cardinal_directions()
{
  return std::vector<Direction>{Dir_N, Dir_E, Dir_S, Dir_W};
};

inline Direction get_45_degrees_clockwise(Direction dir)
{
  return (dir == Dir_MAX ? Dir_MIN : (Direction)(dir + 1));
}

inline Direction get_45_degrees_anticlockwise(Direction dir)
{
  return (dir == Dir_MIN ? Dir_MAX : (Direction)(dir - 1));
}

inline bool in_direction_from_point(xyLoc a, xyLoc b, Direction dir)
{
  xyLoc delta = b - a;
  switch (dir)
  {
    case Dir_N:
      return delta.x == 0 && delta.y > 0;
    case Dir_E:
      return delta.y == 0 && delta.x > 0;
    case Dir_S:
      return delta.x == 0 && delta.y < 0;
    case Dir_W:
      return delta.y == 0 && delta.x < 0;
    case Dir_NE:
      return delta.x == delta.y && delta.x > 0;
    case Dir_SW:
      return delta.x == delta.y && delta.x < 0;
    case Dir_SE:
      return delta.x == -delta.y && delta.x > 0;
    case Dir_NW:
      return delta.x == -delta.y && delta.x < 0;
    default:
      throw std::logic_error("No valid direction given");
  }
}

// Is the direction to travel towards b from a strictly between the directions dir and dir + 45 degrees clockwise? 
inline bool within_45_degrees_clockwise_from_point(xyLoc a, xyLoc b, Direction dir)
{
  xyLoc delta = b - a;
  switch (dir)
  {
    case Dir_N:
      return delta.x > 0 && delta.y > delta.x;
    case Dir_E:
      return delta.y < 0 && delta.y > -delta.x;
    case Dir_S:
      return delta.x < 0 && delta.y < delta.x;
    case Dir_W:
      return delta.y > 0 && delta.y < -delta.x;
    case Dir_NE:
      return within_45_degrees_anticlockwise_from_point(a,b,Dir_E);
    case Dir_SE:
      return within_45_degrees_anticlockwise_from_point(a,b,Dir_S);
    case Dir_SW:
      return within_45_degrees_anticlockwise_from_point(a,b,Dir_W);
    case Dir_NW:
      return within_45_degrees_anticlockwise_from_point(a,b,Dir_N);
    default:
      throw std::logic_error("No valid direction given");
  }
}

inline bool within_45_degrees_anticlockwise_from_point(xyLoc a, xyLoc b, Direction dir)
{
  xyLoc delta = b - a;
  switch (dir)
  {
    case Dir_N:
      return delta.x < 0 && delta.y > -delta.x;
    case Dir_E:
      return delta.y > 0 && delta.y < delta.x;
    case Dir_S:
      return delta.x > 0 && delta.y < -delta.x;
    case Dir_W:
      return delta.y < 0 && delta.y > delta.x;
    case Dir_NE:
      return within_45_degrees_clockwise_from_point(a,b,Dir_N);
    case Dir_SE:
      return within_45_degrees_clockwise_from_point(a,b,Dir_E);
    case Dir_SW:
      return within_45_degrees_clockwise_from_point(a,b,Dir_S);
    case Dir_NW:
      return within_45_degrees_clockwise_from_point(a,b,Dir_W);
    default:
      throw std::logic_error("No valid direction given");
  }
}

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
