#include "NearbyCorners.hpp"

#include <algorithm>

#include "../Graph/Reachable.hpp"
#include "../Utility/SaveLoad.hpp"

void NearbyCorners::find_points_near_corner(corner_index i, const Graph &graph, const CornerVector &corner_vector)
{
  assert (!graph.is_obstacle(corner_vector.get_corner(i)));

  for (map_position p : Reachable::get_safe_reachable_in_all_directions(graph, corner_vector.get_corner(i)))
  {
    if (p != corner_vector.get_corner(i))
    {
      point_to_nearby_corner_indices[p].push_back(i);
    }
  }
}

void NearbyCorners::preprocess(const Graph &graph, const CornerVector &corner_vector)
{
  point_to_nearby_corner_indices.clear();
  point_to_nearby_corner_indices.resize(graph.get_width() * graph.get_height());

  // From every corner, find nearby points, and remember that this corner is near these points.
  corner_index i;
  for (i = 0; i < corner_vector.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << (i != corner_vector.size() - 1 ? ", " : "") << std::flush;
    find_points_near_corner(i, graph, corner_vector);
  }
  if ((i - 1) % 100 != 0)
    std::cout << i - 1;
  std::cout << "\n";
}

void NearbyCorners::save(std::ostream &stream, const Graph &graph, const CornerVector &corner_vector) const
{
  // Save point_to_nearby_corner_indices
  int num_nearby_corner_indices = 0;
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    for (unsigned int corner : point_to_nearby_corner_indices[p])
    {
      SaveLoad::save_unsigned_int_as_binary(stream,corner);
      num_nearby_corner_indices++;
    }
    SaveLoad::save_unsigned_int_as_binary(stream, corner_vector.size());
  }

  std::cout << num_nearby_corner_indices << " nearby corner indices saved" << std::endl;
}

void NearbyCorners::load(std::istream &stream, const Graph &graph, const CornerVector &corner_vector)
{
  point_to_nearby_corner_indices.clear();

  // Load point_to_nearby_corner_indices
  point_to_nearby_corner_indices.resize(graph.num_positions());
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    map_position corner_index = SaveLoad::load_unsigned_int_as_binary(stream);
    while (corner_index != corner_vector.size())
    {
      point_to_nearby_corner_indices[p].push_back(corner_index);
      corner_index = SaveLoad::load_unsigned_int_as_binary(stream);
    }
  }
}

void NearbyCorners::print(corner_index which_index, Printer &printer, const Graph &graph, const CornerVector &corner_vector) const
{
  // print the chosen corner
  printer.add_char('$', graph.loc(corner_vector.get_corner(which_index)));

  // highlight each point with this corner nearby
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    auto nearby_indices = point_to_nearby_corner_indices[p];
    bool is_nearby = std::find(nearby_indices.begin(), nearby_indices.end(), which_index) != nearby_indices.end();
    if (is_nearby)
      printer.add_highlight(Highlight(1), graph.loc(p));
  }
}

void NearbyCorners::print(corner_index which_index, const Graph &graph, const CornerVector &corner_vector) const
{
  Printer printer;
  graph.print(printer);
  print(which_index, printer, graph, corner_vector);
  printer.print();
}

void NearbyCorners::print_all(const Graph &graph, const CornerVector &corner_vector) const
{
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    print(i, graph, corner_vector);
  }
}

