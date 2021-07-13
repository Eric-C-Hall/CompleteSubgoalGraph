#ifndef CORNERGRAPH_PROJECT__DIRECTIONS_HPP
#define CORNERGRAPH_PROJECT__DIRECTIONS_HPP

#include <cassert>
#include <vector>

#include "XYLoc.hpp"
#include "ExactDistance.hpp"

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
  Dir_None = 8
};

enum MidDirection {
  Dir_NNE = 0,
  Dir_ENE = 1,
  Dir_ESE = 2,
  Dir_SSE = 3,
  Dir_SSW = 4,
  Dir_WSW = 5,
  Dir_WNW = 6,
  Dir_NNW = 7,
  Dir_MIDMAX = Dir_NNW,
  Dir_MIDMIN = Dir_NNE
};

enum DivDirection {
  DivDir_N = 0,
  DivDir_NNE = 1,
  DivDir_NE = 2,
  DivDir_ENE = 3,
  DivDir_E = 4,
  DivDir_ESE = 5,
  DivDir_SE = 6,
  DivDir_SSE = 7,
  DivDir_S = 8,
  DivDir_SSW = 9,
  DivDir_SW = 10,
  DivDir_WSW = 11,
  DivDir_W = 12,
  DivDir_WNW = 13,
  DivDir_NW = 14,
  DivDir_NNW = 15,
  DivDir_MAX = DivDir_NNW,
  DivDir_MIN = DivDir_N
};

// ---------------
// Directions
// ---------------

inline int num_directions() {return Dir_MAX - Dir_MIN + 1;}

inline std::vector<Direction> get_cardinal_directions();
inline std::vector<Direction> get_directions();
inline std::vector<Direction> get_diagonal_directions();
inline bool is_cardinal_direction(const Direction dir);

inline Direction get_45_degrees_clockwise(Direction dir);
inline Direction get_45_degrees_anticlockwise(Direction dir);
inline Direction get_opposite_direction(Direction dir);
template <int n>
inline Direction get_n_steps_clockwise(Direction dir);
template <int n>
inline Direction get_n_steps_anticlockwise(Direction dir);

inline Direction get_diagonal_direction_between_points(xyLoc source, xyLoc target);
inline Direction get_straight_direction_between_points(xyLoc source, xyLoc target);

inline bool in_direction_from_point(xyLoc a, xyLoc b, Direction dir);
inline bool within_45_degrees_clockwise_from_point(xyLoc a, xyLoc b, Direction dir);
inline bool within_45_degrees_anticlockwise_from_point(xyLoc a, xyLoc b, Direction dir);

inline exact_distance exact_distance_of_direction(Direction dir);

inline char direction_to_char(const Direction dir);

// ---------------
// MidDirections
// ---------------

inline unsigned int num_middirections() {return Dir_MIDMAX - Dir_MIDMIN + 1;}
inline std::vector<MidDirection> get_middirections();
inline MidDirection get_opposite_middirection(MidDirection middirecion);
inline MidDirection get_middirection_between_points(xyLoc source, xyLoc target);

// ---------------
// DivDirections
// ---------------

inline int num_divdirections() {return DivDir_MAX - DivDir_MIN + 1;}
inline std::vector<DivDirection> get_divdirections();

inline DivDirection get_22_degrees_clockwise(DivDirection dir);
inline DivDirection get_22_degrees_anticlockwise(DivDirection dir);
inline DivDirection get_45_degrees_clockwise(DivDirection dir);
inline DivDirection get_45_degrees_anticlockwise(DivDirection dir);
inline DivDirection get_90_degrees_clockwise(DivDirection dir);
inline DivDirection get_90_degrees_anticlockwise(DivDirection dir);

inline DivDirection get_divdirection_between_points(xyLoc source, xyLoc target);

inline std::vector<DivDirection> get_diagonal_divdirections();

inline bool is_cardinal_direction(DivDirection divdirection);
inline bool is_diagonal_direction(DivDirection divdirection);

inline DivDirection operator+(DivDirection a, DivDirection b) {return (DivDirection)((a + b) % num_divdirections());}

// --------------------------------------------------------

inline std::vector<Direction> get_cardinal_directions()
{
  return std::vector<Direction>{Dir_N, Dir_E, Dir_S, Dir_W};
}

inline std::vector<Direction> get_directions()
{
  return std::vector<Direction>{Dir_N, Dir_NE, Dir_E, Dir_SE, Dir_S, Dir_SW, Dir_W, Dir_NW};
}

inline std::vector<Direction> get_diagonal_directions()
{
  return std::vector<Direction>{Dir_NE, Dir_SE, Dir_SW, Dir_NW};
}

inline std::vector<DivDirection> get_diagonal_divdirections()
{
  return std::vector<DivDirection>{DivDir_NE, DivDir_SE, DivDir_SW, DivDir_NW};
}

inline std::vector<MidDirection> get_middirections()
{
  return std::vector<MidDirection>{Dir_NNE, Dir_ENE, Dir_ESE, Dir_SSE, Dir_SSW, Dir_WSW, Dir_WNW, Dir_NNW};
}

inline std::vector<DivDirection> get_divdirections()
{
  return std::vector<DivDirection>{DivDir_N, DivDir_NNE, DivDir_NE, DivDir_ENE, DivDir_E, DivDir_ESE, DivDir_SE, DivDir_SSE, DivDir_S, DivDir_SSW, DivDir_SW, DivDir_WSW, DivDir_W, DivDir_WNW, DivDir_NW, DivDir_NNW};
}

inline bool is_cardinal_direction(const Direction dir)
{
  return dir == (Direction)(2 * (dir >> 1));
}

inline bool is_cardinal_direction(DivDirection divdirection)
{
  return (divdirection & 0b11) == 0;
}

inline bool is_diagonal_direction(DivDirection divdirection)
{
  return (divdirection & 0b1) == 0 && !is_cardinal_direction(divdirection);
}

inline Direction get_45_degrees_clockwise(Direction dir)
{
  return (dir == Dir_MAX ? Dir_MIN : (Direction)(dir + 1));
}

inline Direction get_45_degrees_anticlockwise(Direction dir)
{
  return (dir == Dir_MIN ? Dir_MAX : (Direction)(dir - 1));
}

inline DivDirection get_22_degrees_clockwise(DivDirection dir)
{
  const int return_value = (dir + 1) % num_divdirections();
  return (DivDirection)(return_value);
}

inline DivDirection get_22_degrees_anticlockwise(DivDirection dir)
{
  const int return_value = (dir + (num_divdirections() - 1)) % num_divdirections();
  return (DivDirection)(return_value);
}

inline DivDirection get_45_degrees_clockwise(DivDirection dir)
{
  const int return_value = (dir + 2) % num_divdirections();
  return (DivDirection)(return_value);
}

inline DivDirection get_45_degrees_anticlockwise(DivDirection dir)
{
  const int return_value = (dir + (num_divdirections() - 2)) % num_divdirections();
  return (DivDirection)(return_value);
}

inline DivDirection get_90_degrees_clockwise(DivDirection dir)
{
  const int return_value = (dir + 4) % num_divdirections();
  return (DivDirection)(return_value);
}

inline DivDirection get_90_degrees_anticlockwise(DivDirection dir)
{
  const int return_value = (dir + (num_divdirections() - 4)) % num_divdirections();
  return (DivDirection)(return_value);
}

inline Direction get_opposite_direction(Direction dir)
{
  return (Direction)(dir >= 4 ?  dir - 4 : dir + 4);
}

inline MidDirection get_opposite_middirection(MidDirection middirecion)
{
  return (MidDirection)(middirecion >= 4 ?  middirecion - 4 : middirecion + 4);
}

template <int n>
inline Direction get_n_steps_clockwise(Direction dir)
{
  int temp = dir + n;
  return (temp <= Dir_MAX ? (Direction)temp : (Direction)(temp - 8));
}

template <int n>
inline Direction get_n_steps_anticlockwise(Direction dir)
{
  return (dir >= n ? (Direction)(dir - n) : (Direction)(dir + 8 - n));
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

inline Direction get_diagonal_direction_between_points(xyLoc source, xyLoc target)
{
  assert(source != target);
  xyLoc delta = target - source;

  if (delta.x > 0)
    if (delta.y > 0)
      return Dir_NE;
    else if (delta.y == 0)
      return Dir_E;
    else // delta.y < 0
      return Dir_SE;
  else if (delta.x == 0)
    if (delta.y > 0)
      return Dir_N;
    else // delta.y <= 0
      return Dir_S;
  else // delta.x < 0
    if (delta.y > 0)
      return Dir_NW;
    else if (delta.y == 0)
      return Dir_W;
    else // delta.y < 0
      return Dir_SW;
}

inline Direction get_straight_direction_between_points(xyLoc source, xyLoc target)
{
  assert(source != target);
  xyLoc delta = target - source;

  if (delta.y > delta.x)
    if (delta.y > -delta.x)
      return Dir_N;
    else if (delta.y == -delta.x)
      return Dir_NW;
    else // delta.y < -delta.x
      return Dir_W;
  else if (delta.y == delta.x)
    if (delta.y > 0)
      return Dir_NE;
    else // delta.y <= 0
      return Dir_SW;
  else // delta.y < delta.x
    if (delta.y > -delta.x)
      return Dir_E;
    else if (delta.y == -delta.x)
      return Dir_SE;
    else // delta.y < -delta.x
      return Dir_S;
}

inline MidDirection get_middirection_between_points(xyLoc source, xyLoc target)
{
  assert(source != target);
  xyLoc delta = target - source;

  if (delta.x > 0)
    if (delta.y > 0)
      if (delta.y > delta.x)
        return Dir_NNE;
      else // delta.y <= delta.x
        return Dir_ENE;
    else // delta.y <= 0
      if (delta.y > -delta.x)
        return Dir_ESE;
      else // delta.y <= -delta.x
        return Dir_SSE; 
  else // delta.x <= 0
    if (delta.y > 0)
      if (delta.y > -delta.x)
        return Dir_NNW;
      else // delta.y <= -delta.x
        return Dir_WNW;
    else // delta.y <= 0
      if (delta.y > delta.x)
        return Dir_WSW;
      else // delta.y <= delta.x
        return Dir_SSW;
}

inline DivDirection get_divdirection_between_points(xyLoc source, xyLoc target)
{
  assert(source != target);
  xyLoc delta = target - source;

  if (delta.x > 0)
    if (delta.y > 0)
      if (delta.y > delta.x)
        return DivDir_NNE;
      else if (delta.y == delta.x)
        return DivDir_NE;
      else // delta.y < delta.x
        return DivDir_ENE;
    else if (delta.y == 0)
      return DivDir_E;
    else // delta.y < 0
      if (delta.y > -delta.x)
        return DivDir_ESE;
      else if (delta.y == -delta.x)
        return DivDir_SE;
      else // delta.y < -delta.x
        return DivDir_SSE;
  else if (delta.x == 0)
    if (delta.y > 0)
      return DivDir_N;
    else // delta.y < 0, since we assume source != target
      return DivDir_S;
  else // delta.x < 0
    if (delta.y > 0)
      if (delta.y > -delta.x)
        return DivDir_NNW;
      else if (delta.y == -delta.x)
        return DivDir_NW;
      else // delta.y < -delta.x
        return DivDir_WNW;
    else if (delta.y == 0)
      return DivDir_W;
    else // delta.y < 0
      if (delta.y > delta.x)
        return DivDir_WSW;
      else if (delta.y == delta.x)
        return DivDir_SW;
      else // delta.y < delta.x
        return DivDir_SSW;
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

inline char direction_to_char(const Direction dir)
{
  switch (dir)
  {
  case Dir_N:
    return 'v';
  case Dir_S:
    return '^';
  case Dir_E:
    return '>';
  case Dir_W:
    return '<';
  case Dir_NE:
    return 'L';
  case Dir_SE:
    return '/';
  case Dir_SW:
    return '\\';
  case Dir_NW:
    return 'S';
  default:
    return '?';
  }
}

#endif
