#ifndef CORNERGRAPH_PROJECT__GRAPH_HPP
#define CORNERGRAPH_PROJECT__GRAPH_HPP

#include <string>
#include <vector>
#include <set>

#define VISIBLE_ONE (2048*2048)
#define VISIBLE_ONE_PLUS  (2048*4*(512+1))
#define VISIBLE_ONE_MINUS (2048*4*(512-1))

#include "MapPosition.hpp"
#include "ExactDistance.hpp"
#include "XYLoc.hpp"

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
};

#endif
