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

// eof
