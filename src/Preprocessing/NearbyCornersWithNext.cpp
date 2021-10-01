#include "NearbyCornersWithNext.hpp"

#include "../Utility/SaveLoad.hpp"
#include "../Utility/Stats.hpp"

void NearbyCornersWithNext::preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners, const RelevantPoints &relevant_points)
{
  int num_with_next = 0;
  int num_without_next = 0;
  point_to_nearby_corner_indices_with_next.clear();
  point_to_nearby_corner_indices_with_next.reserve(graph.num_positions());
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    const auto &nearby_corner_indices = nearby_corners.get_nearby_corner_indices(p);
    std::vector<corner_index> nearby_corner_indices_with_next;
    for (corner_index i : nearby_corner_indices)
    {
      DivDirection incoming_divdirection = get_divdirection_between_points(graph.loc(p), graph.loc(corner_vector.get_corner(i)));
      bool has_next = false;
      for (DivDirection outgoing_divdirection : relevant_points.get_relevant_divdirections(i, incoming_divdirection))
      {
        if (relevant_points.get_relevant_corners(i, outgoing_divdirection).size() > 0)
        {
          has_next = true;
          break;
        }
      }
      if (has_next)
      {
        nearby_corner_indices_with_next.push_back(i);
        num_with_next++;
      }
      else
      {
        num_without_next++;
      }
    }
    point_to_nearby_corner_indices_with_next.push_back(nearby_corner_indices_with_next);
  }
  std::cout << num_with_next << " with next, " << num_without_next << " without next" << std::endl;
}

void NearbyCornersWithNext::remove_collar(const Graph &graph)
{
  int old_width = graph.get_width();
  int old_height = graph.get_height();
  int new_width = old_width-2;
  int new_height = old_height-2;

  std::vector<std::vector<corner_index>> new_with_next;
  new_with_next.resize(new_width * new_height);

  for (int y = 0; y < new_height; y++)
  {
    for (int x = 0; x < new_width; x++)
    {
      new_with_next[y * new_width + x] = point_to_nearby_corner_indices_with_next[graph.pos(x+1,y+1)];
    }
  }

  point_to_nearby_corner_indices_with_next.swap(new_with_next);
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

void NearbyCornersWithNext::print_stats(const Graph &graph) const
{
  std::vector<int> num_nearby_with_next_vec;
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    if (graph.is_obstacle(p))
      continue;
    num_nearby_with_next_vec.push_back(get_nearby_corner_indices_with_next(p).size());
  }

  std::vector<int> num_nearby_with_next_hist = Stats::to_histogram(num_nearby_with_next_vec);

  std::cout << "Num nearby corners with next histogram: " << std::flush;
  Stats::print_histogram(num_nearby_with_next_hist);
  std::cout << std::endl;

  std::cout << "Num nearby corners with next stats: " << std::endl;
  auto num_nearby_with_next_stats = Stats::get_stats(num_nearby_with_next_vec);
  Stats::print_stats(num_nearby_with_next_stats);
  std::cout << std::endl;
}

