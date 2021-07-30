#include "CornerVectorSplittable.hpp"

#include <algorithm>

void CornerVectorSplittable::preprocess(const Graph &graph, const CornerVector &corner_vector, const RelevantPoints &relevant_points)
{
  corners_splittable.clear();
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    // Find directions in which there is no next corner, and opposites of directions with next corners
    std::vector<DivDirection> no_next_directions;
    no_next_directions.reserve(num_divdirections());
    std::vector<DivDirection> opposite_of_with_next_directions;
    opposite_of_with_next_directions.reserve(num_divdirections());
    for (DivDirection dir : get_divdirections())
    {
      if (relevant_points.get_relevant_corners(i, dir).size() == 0)
      {
        no_next_directions.push_back(dir);
      }
      else
      {
        opposite_of_with_next_directions.push_back(get_opposite_direction(dir));
      }
    }
    
    // Check that all directions relevant to opposites of directions with next corners have no next corner
    for (DivDirection opp_dir : opposite_of_with_next_directions)
    {
      for (DivDirection check_dir : relevant_points.get_relevant_divdirections(i, opp_dir))
      {
        if (std::find(std::begin(no_next_directions), std::end(no_next_directions), check_dir) == no_next_directions.end())
        {
          goto end_of_iteration_for_this_corner_index;
        }
      }
    }

    // If we reach here, the opposite of all directions without next corners is only relevant to directions with next corners
    corners_splittable.push_back(corner_vector.get_corner(i));
    continue;

    end_of_iteration_for_this_corner_index:;
  }

  std::cout << corners_splittable.size() << "/" << corner_vector.size() << " corners have no next corner in important directions" << std::endl;
}

void CornerVectorSplittable::print(Printer &printer, const Graph &graph) const
{
  for (map_position c : corners_splittable)
  {
    printer.add_char('s', graph.loc(c));
  }
}
