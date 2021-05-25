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

// top left is zero

// NOTE: Graph has a collar of obstacles added to it.
// Obstacles are added around all the edges.
class Graph {
  private:
  unsigned int _width;
  unsigned int _height;
  std::vector<bool> _obstacles;

  // Used by load_map, you should call load_map instead
  void read_width_height(FILE * f);
  void read_map(FILE * f);
  void add_collar();
  void read_file(FILE *f);

  public:
  Graph();

  void load_map(const char *fname);

  std::vector<std::pair<map_position, exact_distance>> adjacent_locations_and_dists(map_position p) const;  
  bool safe_reachable(map_position a, map_position b) const;

  inline bool is_obstacle(map_position p) const {return _obstacles[p];}

  inline unsigned int y(map_position p) const {return p / get_width();}
  inline unsigned int x(map_position p) const {return p % get_width();}
  // TODO: maybe this can be done without converting to xyLoc? Maybe not though
  inline exact_distance octile_distance(map_position a, map_position b) const
  {
    unsigned int x_a = x(a);
    unsigned int x_b = x(b);
    unsigned int y_a = y(a);
    unsigned int y_b = y(b);

    unsigned int abs_x_diff = (x_a > x_b ? x_a - x_b : x_b - x_a);
    unsigned int abs_y_diff = (y_a > y_b ? y_a - y_b : y_b - y_a);

    int num_diagonal = std::min(abs_x_diff, abs_y_diff);
    int num_straight = std::max(abs_x_diff, abs_y_diff) - num_diagonal;
    return exact_distance(num_straight, num_diagonal);
  }

  inline map_position up(map_position p) const {return p + get_width();}
  inline map_position down(map_position p) const {return p - get_width();}
  inline map_position left(map_position p) const {return p - 1;}
  inline map_position right(map_position p) const {return p + 1;}
  
  inline map_position translate_x(map_position p, int num) const {return p + num;}
  inline map_position translate_y(map_position p, int num) const {return p + num * get_width();}

  inline map_position num_positions() const {return _obstacles.size();}

  inline unsigned int get_width() const {return _width;}
  inline unsigned int get_height() const {return _height;}

  inline map_position pos(const unsigned int x, const unsigned int y) const {return y*get_width() + x;}
  inline map_position pos(const xyLoc &loc) const {return pos(loc.x, loc.y);}
  inline xyLoc loc(map_position p) const {return xyLoc(x(p), y(p));}

  void get_safe_reachable_in_directions(std::vector<map_position> &output, const map_position origin, const Direction straight_direction, const Direction diagonal_direction, const int num_step_bound = INT_MAX) const;
  std::vector<map_position> get_safe_reachable_in_all_directions(const map_position origin) const;

  std::pair<xyLoc, xyLoc> get_bounds_of_points(const std::vector<map_position> &v) const;
  inline std::pair<xyLoc, xyLoc> get_bounds_of_points(const xyLoc a, const xyLoc b) const;
};

inline std::pair<xyLoc, xyLoc> Graph::get_bounds_of_points(const xyLoc a, const xyLoc b) const
{
  const xyLoc lower_bound = xyLoc(std::min(a.x, b.x), std::min(a.y, b.y));
  const xyLoc upper_bound = xyLoc(std::max(a.x, b.x), std::max(a.y, b.y));
  return std::pair<xyLoc, xyLoc>(lower_bound, upper_bound);
}

inline void moving_direction(unsigned int dir, map_position & pos, const Graph &graph);

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

#endif
