#include "NearbyCornersWithNext.hpp"

#include "../Utility/SaveLoad.hpp"

void NearbyCornersWithNext::preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners, const RelevantPoints &relevant_points)
{
  point_to_nearby_corner_indices_with_next.clear();
  point_to_nearby_corner_indices_with_next.reserve(graph.num_positions());
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    const auto &nearby_corner_indices = nearby_corners.get_nearby_corner_indices(p);
    std::vector<corner_index> nearby_corner_indices_with_next;
    for (corner_index i : nearby_corner_indices)
    {
      DivDirection divdirection = get_divdirection_between_points(graph.loc(p), graph.loc(corner_vector.get_corner(i)));
      if (relevant_points.get_relevant_corners(i, divdirection).size() > 0)
      {
        nearby_corner_indices_with_next.push_back(i);
      }
    }
    point_to_nearby_corner_indices_with_next.push_back(nearby_corner_indices_with_next);
  }
}

void NearbyCornersWithNext::save(std::ostream &stream, const Graph &graph, const CornerVector &corner_vector) const
{
  int num_nearby_corner_indices_with_next = 0;
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    for (corner_index i : point_to_nearby_corner_indices_with_next[p])
    {
      SaveLoad::save_as_binary(stream,i);
      num_nearby_corner_indices_with_next++;
    }
    SaveLoad::save_as_binary(stream, (corner_index)(corner_vector.size()));
  }

  std::cout << num_nearby_corner_indices_with_next << " nearby corner indices with next saved" << std::endl;
}

void NearbyCornersWithNext::load(std::istream &stream, const Graph &graph, const CornerVector &corner_vector)
{
  point_to_nearby_corner_indices_with_next.clear();

  // Load point_to_nearby_corner_indices
  point_to_nearby_corner_indices_with_next.resize(graph.num_positions());
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    corner_index i;
    SaveLoad::load_as_binary(stream, i);
    while (i != corner_vector.size())
    {
      point_to_nearby_corner_indices_with_next[p].push_back(i);
      SaveLoad::load_as_binary(stream, i);
    }
  }
}

void NearbyCornersWithNext::print_nearby_with_next(map_position pos, Printer &printer, const Graph &graph, const CornerVector &corner_vector) const
{
  printer.add_highlight(Highlight(2), graph.loc(pos));
  for (corner_index i : point_to_nearby_corner_indices_with_next[pos])
  {
    printer.add_highlight(Highlight(3), graph.loc(corner_vector.get_corner(i)));
  }
}

