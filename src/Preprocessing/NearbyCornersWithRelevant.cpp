#include "NearbyCornersWithRelevant.hpp"

#include "../Utility/SaveLoad.hpp"

void NearbyCornersWithRelevant::preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners, const RelevantPoints &relevant_points)
{
  int num_with_relevant = 0;
  int num_without_relevant = 0;
  point_to_nearby_corner_indices_with_relevant.clear();
  point_to_nearby_corner_indices_with_relevant.reserve(graph.num_positions());
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    const auto &nearby_corner_indices = nearby_corners.get_nearby_corner_indices(p);
    std::vector<corner_index> nearby_corner_indices_with_relevant;
    for (corner_index i : nearby_corner_indices)
    {
      DivDirection incoming_divdirection = get_divdirection_between_points(graph.loc(p), graph.loc(corner_vector.get_corner(i)));
      for (DivDirection outgoing_divdirection : relevant_points.get_relevant_divdirections(i, incoming_divdirection))
      {
        if (relevant_points.get_relevant_points(i, outgoing_divdirection).size() > 0)
        {
          nearby_corner_indices_with_relevant.push_back(i);
          num_with_relevant++;
        }
        else
        {
          num_without_relevant++;
        }
      }
    }
    point_to_nearby_corner_indices_with_relevant.push_back(nearby_corner_indices_with_relevant);
  }
  std::cout << num_with_relevant << " with relevant, " << num_without_relevant << " without relevant" << std::endl;
}

void NearbyCornersWithRelevant::remove_collar(const Graph &graph)
{
  int old_width = graph.get_width();
  int old_height = graph.get_height();
  int new_width = old_width-2;
  int new_height = old_height-2;

  std::vector<std::vector<corner_index>> new_with_relevant;
  new_with_relevant.resize(new_width * new_height);

  for (int y = 0; y < new_height; y++)
  {
    for (int x = 0; x < new_width; x++)
    {
      new_with_relevant[y * new_width + x] = point_to_nearby_corner_indices_with_relevant[graph.pos(x+1,y+1)];
    }
  }

  point_to_nearby_corner_indices_with_relevant.swap(new_with_relevant);
}

void NearbyCornersWithRelevant::save(std::ostream &stream, const Graph &graph, const CornerVector &corner_vector) const
{
  int num_nearby_corner_indices_with_relevant = 0;
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    for (corner_index i : point_to_nearby_corner_indices_with_relevant[p])
    {
      SaveLoad::save_as_binary(stream,i);
      num_nearby_corner_indices_with_relevant++;
    }
    SaveLoad::save_as_binary(stream, (corner_index)(corner_vector.size()));
  }

  std::cout << num_nearby_corner_indices_with_relevant << " nearby corner indices with relevant saved" << std::endl;
}

void NearbyCornersWithRelevant::load(std::istream &stream, const Graph &graph, const CornerVector &corner_vector)
{
  point_to_nearby_corner_indices_with_relevant.clear();

  // Load point_to_nearby_corner_indices
  point_to_nearby_corner_indices_with_relevant.resize(graph.num_positions());
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    corner_index i;
    SaveLoad::load_as_binary(stream, i);
    while (i != corner_vector.size())
    {
      point_to_nearby_corner_indices_with_relevant[p].push_back(i);
      SaveLoad::load_as_binary(stream, i);
    }
  }
}

void NearbyCornersWithRelevant::print_nearby_with_relevant(map_position pos, Printer &printer, const Graph &graph, const CornerVector &corner_vector) const
{
  printer.add_highlight(Highlight(2), graph.loc(pos));
  for (corner_index i : point_to_nearby_corner_indices_with_relevant[pos])
  {
    printer.add_highlight(Highlight(3), graph.loc(corner_vector.get_corner(i)));
  }
}

