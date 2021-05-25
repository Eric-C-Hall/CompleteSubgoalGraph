#include <algorithm>
#include <assert.h>
#include <fstream>
#include <iostream>

#include "Graph.hpp"

Graph::Graph()
{
}

void Graph::read_width_height(FILE * f)
{
  int num_args_filled = fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &_height, &_width);

  _width += 2;
  _height += 2;

  if (num_args_filled != 2)
  {
    throw std::runtime_error("Error finding height/width when scanning file");
  }
}

// Assumes _width and _height are known
void Graph::read_map(FILE * f)
{
  _obstacles.resize(get_width()*get_height());

  // Note: we don't include x = 0, y = 0, x = width - 1, y = height - 1, since
  // those will be obstacles in the collar
  for (unsigned int y = 1; y < get_height() - 1; y++)
  {
    for (unsigned int x = 1; x < get_width() - 1; x++)
    {
      char c;
      do {
        int num_args_filled = fscanf(f, "%c", &c);
        if (num_args_filled != 1)
        {
          throw std::runtime_error("Error finding character describing map");
        }

      } while (isspace(c));
      _obstacles[pos(x,y)] = !(c == '.' || c == 'G' || c == 'S');
      //printf("%c", c);
    }
    //printf("\n");
  }
}

void Graph::add_collar()
{
  for (unsigned int x = 0; x < get_width(); x++)
  {
    _obstacles[pos(x, 0)] = true;
    _obstacles[pos(x, get_height() - 1)] = true;
  }

  for (unsigned int y = 0; y < get_height(); y++)
  {
    _obstacles[pos(0, y)] = true;
    _obstacles[pos(get_width() -1, y)] = true;
  }
}

void Graph::read_file(FILE *f)
{
  read_width_height(f);
  read_map(f);
  add_collar();
}

void Graph::load_map(const char *fname)
{
  FILE *f;
  f = fopen(fname, "r");
  if (f)
  {
    read_file(f);
    fclose(f);
  }
  else
  {
    throw std::runtime_error("Error opening file");
  }
}

std::vector<std::pair<map_position, exact_distance>> Graph::adjacent_locations_and_dists(map_position p) const
{
  std::vector<std::pair<map_position, exact_distance>> return_value;
  return_value.reserve(8);

  return_value.emplace_back(up(p), STRAIGHT_EXACT_DISTANCE);
  return_value.emplace_back(up(right(p)), DIAGONAL_EXACT_DISTANCE);
  return_value.emplace_back(right(p), STRAIGHT_EXACT_DISTANCE);
  return_value.emplace_back(right(down(p)), DIAGONAL_EXACT_DISTANCE);
  return_value.emplace_back(down(p), STRAIGHT_EXACT_DISTANCE);
  return_value.emplace_back(down(left(p)), DIAGONAL_EXACT_DISTANCE);
  return_value.emplace_back(left(p), STRAIGHT_EXACT_DISTANCE);
  return_value.emplace_back(left(up(p)), DIAGONAL_EXACT_DISTANCE);

  return return_value;
}

template<bool is_y_long, bool is_long_positive, bool is_short_positive>
bool check_parallelogram_clear(xyLoc start, const int long_distance, const int short_distance, const Graph &graph)
{

  for (int i = 0; i < short_distance; i++)
  {
    for (int j = 0; j < long_distance; j++)
    {
      xyLoc check_loc;
      
      int translate_short = (is_short_positive ? i : -i);
      int translate_long = (is_long_positive ? i + j : -i-j);
      
      if (is_y_long)
        check_loc = xyLoc(start.x + translate_short, start.y + translate_long);
      else
        check_loc = xyLoc(start.x + translate_long, start.y + translate_short);
        
      if (graph.is_obstacle(graph.pos(check_loc)))
        return false;
    }
  }

  return true;
}

bool safe_reachable_xyloc(xyLoc a, xyLoc b, const Graph &graph)
{
  int x_diff = b.x - a.x;
  int y_diff = b.y - a.y;
  
  int abs_x_diff = std::abs(x_diff);
  int abs_y_diff = std::abs(y_diff);
  
  if (abs_y_diff >= abs_x_diff)
    if (y_diff >= 0)
      if (x_diff >= 0)
        return check_parallelogram_clear<true, true, true>(a, abs_y_diff - abs_x_diff + 1, abs_x_diff + 1, graph);
      else
        return check_parallelogram_clear<true, true, false>(a, abs_y_diff - abs_x_diff + 1, abs_x_diff + 1, graph);
    else
      if (x_diff >= 0)
        return check_parallelogram_clear<true, false, true>(a, abs_y_diff - abs_x_diff + 1, abs_x_diff + 1, graph);
      else
        return check_parallelogram_clear<true, false, false>(a, abs_y_diff - abs_x_diff + 1, abs_x_diff + 1, graph);
  else
    if (y_diff >= 0)
      if (x_diff >= 0)
        return check_parallelogram_clear<false, true, true>(a, abs_x_diff - abs_y_diff + 1, abs_y_diff + 1, graph);
      else
        return check_parallelogram_clear<false, false, true>(a, abs_x_diff - abs_y_diff + 1, abs_y_diff + 1, graph);
    else
      if (x_diff >= 0)
        return check_parallelogram_clear<false, true, false>(a, abs_x_diff - abs_y_diff + 1, abs_y_diff + 1, graph);
      else
        return check_parallelogram_clear<false, false, false>(a, abs_x_diff - abs_y_diff + 1, abs_y_diff + 1, graph);
    
}

bool Graph::safe_reachable(map_position a, map_position b) const
{
  return safe_reachable_xyloc(loc(a), loc(b), *this);
}

// Hitting an obstacle decreases num_step_bound, to ensure that all points are safe-reachable rather than just octile-reachable
// num_step_bound starts as INT_MAX by default
//
// Note: origin is included, and if doing this multiple times in multiple direction pairs, may have duplicates.
void Graph::get_safe_reachable_in_directions(std::vector<map_position> &output, const map_position origin, const Direction straight_direction, const Direction diagonal_direction, const int num_step_bound) const
{
  map_position curr_pos = origin;
  int num_steps = 0;

  while (!is_obstacle(curr_pos) && num_steps < num_step_bound)
  {
    output.push_back(curr_pos);

    moving_direction(straight_direction, curr_pos, *this);
    num_steps++;
  }

  if (num_steps != 0)
  {
    map_position new_origin = origin;
    moving_direction(diagonal_direction, new_origin, *this);
    get_safe_reachable_in_directions(output, new_origin, straight_direction, diagonal_direction, num_steps);
  }
}

// Note: origin is included
std::vector<map_position> Graph::get_safe_reachable_in_all_directions(const map_position origin) const
{
  std::vector<map_position> return_value;

  get_safe_reachable_in_directions(return_value, origin, Dir_N, Dir_NW);
  get_safe_reachable_in_directions(return_value, origin, Dir_N, Dir_NE);
  get_safe_reachable_in_directions(return_value, origin, Dir_E, Dir_NE);
  get_safe_reachable_in_directions(return_value, origin, Dir_E, Dir_SE);
  get_safe_reachable_in_directions(return_value, origin, Dir_S, Dir_SE);
  get_safe_reachable_in_directions(return_value, origin, Dir_S, Dir_SW);
  get_safe_reachable_in_directions(return_value, origin, Dir_W, Dir_SW);
  get_safe_reachable_in_directions(return_value, origin, Dir_W, Dir_NW);

  // May have duplicates, since the same point may be reachable for multiple
  // direction pairs, for example the origin is reachable for all direction
  // pairs. The other example is that points purely in the main_direction are
  // reachable regardless of the secondary direction.
  std::sort(return_value.begin(), return_value.end());
  const auto last = std::unique(return_value.begin(), return_value.end());
  return_value.erase(last, return_value.end());
  
  return return_value;
}

std::pair<xyLoc, xyLoc> Graph::get_bounds_of_points(const std::vector<map_position> &v) const
{
  assert(v.size() > 0);
  xyLoc lower_bound =  loc(v.front());
  xyLoc upper_bound = lower_bound;
  for (map_position p : v)
  {
    xyLoc l = loc(p);
    if (l.x < lower_bound.x)
      lower_bound.x = l.x;
    else if (l.x > upper_bound.x)
      upper_bound.x = l.x;

    if (l.y < lower_bound.y)
      lower_bound.y = l.y;
    else if (l.y > upper_bound.y)
      upper_bound.y = l.y;
  }

  return std::pair<xyLoc, xyLoc>(lower_bound, upper_bound);
}

// eof
