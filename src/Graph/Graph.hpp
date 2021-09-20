#ifndef CORNERGRAPH_PROJECT__GRAPH_HPP
#define CORNERGRAPH_PROJECT__GRAPH_HPP

#include <string>
#include <vector>
#include <set>
#include <climits>

#include "MapPosition.hpp"
#include "ExactDistance.hpp"
#include "XYLoc.hpp"
#include "Directions.hpp"
#include "../Visualise/Printer.hpp"

// top left is zero

// NOTE: Graph has a collar of obstacles added to it.
// Obstacles are added around all the edges.
class Graph {
  private:
  unsigned int _width;
  unsigned int _height;
  std::vector<bool> _obstacles;

  void add_collar();

  public:
  Graph();

  void load_bits_without_collar(const std::vector<bool> &bits, unsigned int input_width, unsigned int input_height);
  void load_bits_with_collar(const std::vector<bool> &bits, unsigned int input_width, unsigned int input_height);

  void remove_collar();

  std::vector<std::pair<map_position, exact_distance>> adjacent_locations_and_dists(map_position p) const;  

  inline bool is_obstacle(map_position p) const {return _obstacles[p];}

  inline unsigned int y(map_position p) const {return p / get_width();}
  inline unsigned int x(map_position p) const {return p % get_width();}

  inline map_position up(map_position p) const {return p + get_width();}
  inline map_position down(map_position p) const {return p - get_width();}
  inline map_position left(map_position p) const {return p - 1;}
  inline map_position right(map_position p) const {return p + 1;}
  inline map_position step_in_direction(map_position p, Direction dir) const;
  inline map_position step_in_direction(map_position l, DivDirection divdirection) const;
  inline map_position translate_x(map_position p, int num) const {return p + num;}
  inline map_position translate_y(map_position p, int num) const {return p + num * get_width();}

  inline bool up_possible(map_position p) const {return y(p) != get_height() - 1;}
  inline bool down_possible(map_position p) const {return y(p) != 0;}
  inline bool left_possible(map_position p) const {return x(p) != 0;}
  inline bool right_possible(map_position p) const {return x(p) != get_width() - 1;}
  inline bool is_corner_cut(map_position p, map_position q) const {return is_obstacle(pos(x(p), y(q))) || is_obstacle(pos(x(q),y(p)));}

  inline map_position num_positions() const {return _obstacles.size();}

  inline unsigned int get_width() const {return _width;}
  inline unsigned int get_height() const {return _height;}

  inline map_position pos(const unsigned int x, const unsigned int y) const {return y*get_width() + x;}
  inline map_position pos(const xyLoc &loc) const {return pos(loc.x, loc.y);}
  inline xyLoc loc(map_position p) const {return make_xyLoc(x(p), y(p));}

  inline std::pair<xyLoc, xyLoc> get_bounds_of_points(const std::vector<map_position> &v) const;
  inline std::pair<xyLoc, xyLoc> get_bounds_of_points(const xyLoc a, const xyLoc b) const;
  inline std::pair<xyLoc, xyLoc> get_bounds_of_points(const std::pair<xyLoc, xyLoc> bounds, const xyLoc c) const;
  inline std::pair<xyLoc, xyLoc> get_bounds_of_points(const std::pair<xyLoc, xyLoc> a, const std::pair<xyLoc, xyLoc> b) const;

  inline bool is_point_in_bounds(const xyLoc l, const std::pair<xyLoc, xyLoc> bounds) const;

  inline bool adjacent(map_position a, map_position b) const;

  void debug_cut_sides(int xmin, int xmax, int ymin, int ymax);

  void print(Printer &printer) const;
  void print() const;
};

inline std::pair<xyLoc, xyLoc> Graph::get_bounds_of_points(const xyLoc a, const xyLoc b) const
{
  const xyLoc lower_bound = make_xyLoc(std::min(a.x, b.x), std::min(a.y, b.y));
  const xyLoc upper_bound = make_xyLoc(std::max(a.x, b.x), std::max(a.y, b.y));
  return std::pair<xyLoc, xyLoc>(lower_bound, upper_bound);
}

inline std::pair<xyLoc, xyLoc> Graph::get_bounds_of_points(const std::vector<map_position> &v) const
{
  assert(v.size() > 0);

  std::pair<xyLoc, xyLoc> bounds;
  bounds.first = loc(v.front());
  bounds.second = bounds.first;

  for (map_position p : v)
  {
    bounds = get_bounds_of_points(bounds, loc(p));
  }

  return bounds;
}

inline std::pair<xyLoc, xyLoc> Graph::get_bounds_of_points(const std::pair<xyLoc, xyLoc> bounds, const xyLoc c) const
{
  xyLoc new_lower_bound = make_xyLoc(std::min(c.x, bounds.first.x), std::min(c.y, bounds.first.y));
  xyLoc new_upper_bound = make_xyLoc(std::max(c.x, bounds.second.x), std::max(c.y, bounds.second.y));
  return std::pair<xyLoc, xyLoc>(new_lower_bound, new_upper_bound);
}

inline std::pair<xyLoc, xyLoc> Graph::get_bounds_of_points(const std::pair<xyLoc, xyLoc> a, const std::pair<xyLoc, xyLoc> b) const
{
  xyLoc new_lower_bound = make_xyLoc(std::min(a.first.x, b.first.x), std::min(a.first.y, b.first.y));
  xyLoc new_upper_bound = make_xyLoc(std::max(a.second.x, b.second.x), std::max(a.second.y, b.second.y));
  return std::pair<xyLoc, xyLoc>(new_lower_bound, new_upper_bound);
}

inline bool Graph::is_point_in_bounds(const xyLoc l, const std::pair<xyLoc, xyLoc> bounds) const
{
  return l.x >= bounds.first.x && l.y >= bounds.first.y && l.x <= bounds.second.x && l.y <= bounds.second.y;
}

inline void moving_direction(unsigned int dir, map_position & pos, const Graph &graph);
inline map_position move_in_middirection(const MidDirection middirection, const map_position pos, const Graph &graph);

inline map_position Graph::step_in_direction(map_position p, Direction dir) const
{
  moving_direction(dir, p, *this);
  return p;
}

inline map_position Graph::step_in_direction(map_position p, DivDirection divdirection) const
{
  switch (divdirection)
  {
    case DivDir_NNE:
    case DivDir_NNW:
      p = up(p);
      // No break here is deliberate
    case DivDir_N:
    case DivDir_NE:
    case DivDir_NW:
    case DivDir_ENE:
    case DivDir_WNW:
      p = up(p);
      break;
    case DivDir_SSE:
    case DivDir_SSW:
      p = down(p);
      // No break here is deliberate
    case DivDir_S:
    case DivDir_SE:
    case DivDir_SW:
    case DivDir_ESE:
    case DivDir_WSW:
      p = down(p);
      break;
    default:
      break;
  }

  switch (divdirection)
  {
    case DivDir_ENE:
    case DivDir_ESE:
      p = right(p);
      // No break here is deliberate
    case DivDir_E:
    case DivDir_NE:
    case DivDir_SE:
    case DivDir_NNE:
    case DivDir_SSE:
      p = right(p);
      break;
    case DivDir_WNW:
    case DivDir_WSW:
      p = left(p);
      // No break here is deliberate
    case DivDir_W:
    case DivDir_NW:
    case DivDir_SW:
    case DivDir_NNW:
    case DivDir_SSW:
      p = left(p);
      break;
    default:
      break;
  }

  return p;
}

// TODO: I think this should just return a map_position, rather than modifying the input map_position
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

inline map_position move_in_middirection(const MidDirection middirection, const map_position pos, const Graph &graph)
{
  switch (middirection)
  {
    case Dir_NNW:
      return graph.up(graph.up(graph.left(pos)));
    case Dir_NNE:
      return graph.up(graph.up(graph.right(pos)));
    case Dir_ENE:
      return graph.right(graph.right(graph.up(pos)));
    case Dir_ESE:
      return graph.right(graph.right(graph.down(pos)));
    case Dir_SSE:
      return graph.down(graph.down(graph.right(pos)));
    case Dir_SSW:
      return graph.down(graph.down(graph.left(pos)));
    case Dir_WSW:
      return graph.left(graph.left(graph.down(pos)));
    case Dir_WNW:
      return graph.left(graph.left(graph.up(pos)));
    default:
      return pos;
  }
}

inline bool Graph::adjacent(map_position a, map_position b) const
{
  xyLoc loc_a = loc(a);
  xyLoc loc_b = loc(b);
  return std::abs(loc_a.x - loc_b.x) <= 1 && std::abs(loc_a.y - loc_b.y) <= 1;
}

#endif
