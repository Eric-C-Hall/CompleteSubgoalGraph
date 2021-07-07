#include "CornerVector.hpp"

#include "../Utility/SaveLoad.hpp"

bool CornerVector::calculate_is_corner(map_position p, const Graph &graph)
{
  // Obstacles cannot be corners
  if (graph.is_obstacle(p))
  {
    return false;
  }

  // Check each diagonal direction to see if this is a corner for that direction
  for (Direction dir : get_diagonal_directions())
  {
    Direction clock_dir = get_45_degrees_clockwise(dir);
    Direction anticlock_dir = get_45_degrees_anticlockwise(dir);

    map_position diag_pos = graph.step_in_direction(p, dir);
    map_position clock_pos = graph.step_in_direction(p, clock_dir);
    map_position anticlock_pos = graph.step_in_direction(p, anticlock_dir);

    if (graph.is_obstacle(diag_pos) && !graph.is_obstacle(clock_pos) && !graph.is_obstacle(anticlock_pos))
    {
      return true;
    }
  }

  // If not a corner for any diagonal direction, then it is not a corner
  return false;
}

void CornerVector::preprocess(const Graph &graph)
{
  corners.clear();
  for (unsigned int x=1 ; x<graph.get_width()-1 ; x++)
  {
    for (unsigned int y=1 ; y<graph.get_height()-1 ; y++)
    {
      map_position p = graph.pos(x,y);
      if (calculate_is_corner(p, graph))
      {
        corners.push_back(p);
      }
    }
  }

  std::cout << "Number of corners: " << corners.size() << std::endl;
}

void CornerVector::save(std::ostream &stream) const
{
  SaveLoad::save_unsigned_int_as_binary(stream, corners.size());

  // Save corners
  int num_corners = 0;
  for (unsigned int corner : corners)
  {
    SaveLoad::save_unsigned_int_as_binary(stream, corner);
    num_corners++;
  }

  std::cout << num_corners << " corners saved" << std::endl;
}

void CornerVector::load(std::istream &stream)
{
  corners.clear();

  unsigned int num_corners = SaveLoad::load_unsigned_int_as_binary(stream);

  // Load corners
  corners.reserve(num_corners);
  for (unsigned int i = 0; i < num_corners; i++)
  {
    map_position corner_pos = SaveLoad::load_unsigned_int_as_binary(stream);
    corners.push_back(corner_pos);
  }
}

void CornerVector::print(const Graph &graph)
{
  
}

