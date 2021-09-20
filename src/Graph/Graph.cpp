#include <algorithm>
#include <assert.h>
#include <fstream>
#include <iostream>

#include "Graph.hpp"

Graph::Graph()
{
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


void Graph::load_bits_without_collar(const std::vector<bool> &input_bits, unsigned int input_width, unsigned int input_height)
{
  _width = input_width;
  _height = input_height;
  
  _obstacles.resize(get_width()*get_height());

  for (unsigned int y = 0; y < get_height(); y++)
  {
    for (unsigned int x = 0; x < get_width(); x++)
    {
      _obstacles[pos(x,y)] = !input_bits[pos(x,y)];
    }
  }
}

void Graph::load_bits_with_collar(const std::vector<bool> &input_bits, unsigned int input_width, unsigned int input_height)
{
  _width = input_width;
  _height = input_height;

  _width += 2;
  _height += 2;

  _obstacles.resize(get_width()*get_height());

  for (unsigned int y = 1; y < get_height() - 1; y++)
  {
    for (unsigned int x = 1; x < get_width() - 1; x++)
    {
      _obstacles[pos(x,y)] = !input_bits[(y - 1) * (get_width() - 2) + (x - 1)];
    }
  }

  add_collar();
}

void Graph::remove_collar()
{
  int old_width = get_width();
  int old_height = get_height();
  int new_width = old_width-2;
  int new_height = old_height-2;

  std::vector<bool> new_obstacles;
  new_obstacles.resize(new_width * new_height);

  for (int y = 0; y < new_height; y++)
  {
    for (int x = 0; x < new_width; x++)
    {
      new_obstacles[y * new_width + x] = _obstacles[pos(x+1,y+1)];
    }
  }

  _obstacles.swap(new_obstacles);
}

// TODO: A little messy
std::vector<std::pair<map_position, exact_distance>> Graph::adjacent_locations_and_dists(map_position p) const
{
  std::vector<std::pair<map_position, exact_distance>> return_value;
  return_value.reserve(8);

  if (up_possible(p))
    return_value.emplace_back(up(p), STRAIGHT_EXACT_DISTANCE);

  if (up_possible(p) && right_possible(p) && !is_corner_cut(p, up(right(p))))
    return_value.emplace_back(up(right(p)), DIAGONAL_EXACT_DISTANCE);

  if (right_possible(p))
    return_value.emplace_back(right(p), STRAIGHT_EXACT_DISTANCE);

  if (right_possible(p) && down_possible(p) && !is_corner_cut(p, right(down(p))))
    return_value.emplace_back(right(down(p)), DIAGONAL_EXACT_DISTANCE);

  if (down_possible(p))
    return_value.emplace_back(down(p), STRAIGHT_EXACT_DISTANCE);

  if (down_possible(p) && left_possible(p) && !is_corner_cut(p, down(left(p))))
    return_value.emplace_back(down(left(p)), DIAGONAL_EXACT_DISTANCE);

  if (left_possible(p))
    return_value.emplace_back(left(p), STRAIGHT_EXACT_DISTANCE);

  if (left_possible(p) && up_possible(p) && !is_corner_cut(p, left(up(p))))
    return_value.emplace_back(left(up(p)), DIAGONAL_EXACT_DISTANCE);

  return return_value;
}

void Graph::debug_cut_sides(int xlowercut, int xuppercut, int ylowercut, int yuppercut)
{
  std::cout << "-----------------------------------------------------------" << std::endl;
  std::cout << "WARNING: For the purposes of debug, the sides are being cut" << std::endl;
  std::cout << "-----------------------------------------------------------" << std::endl;

  unsigned int old_width = get_width();
  unsigned int old_height = get_height();

  assert ((int)get_width() > xlowercut + xuppercut);
  unsigned int new_width = get_width();
  new_width -= xlowercut;
  new_width -= xuppercut; 

  assert ((int)get_height() > ylowercut + yuppercut);
  unsigned int new_height = get_height();
  new_height -= ylowercut;
  new_height -= yuppercut;

  std::vector<bool> new_obstacles;
  new_obstacles.reserve(new_width * new_height);

  for (unsigned int y = ylowercut; y < old_height - yuppercut; y++)
  {
    for (unsigned int x = xlowercut; x < old_width - xuppercut; x++)
    {
      new_obstacles.push_back(is_obstacle(pos(x,y)));
    }
  }
  _obstacles.swap(new_obstacles);

  _width = new_width;
  _height = new_height;
  add_collar();
}

void Graph::print(Printer &printer) const
{
  for (map_position p = 0; p < num_positions(); p++)
  {
    xyLoc l = loc(p);
    char c = (is_obstacle(p) ? '@' : '-');
    printer.add_char(c, l);
  }
}

void Graph::print() const
{
  Printer printer;
  print(printer);
  printer.print();
}

// eof
