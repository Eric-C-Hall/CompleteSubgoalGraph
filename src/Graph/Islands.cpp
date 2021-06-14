#include "Islands.hpp"

void Islands::create_island_from_point(map_position p, int curr_island_index)
{
  std::pair<xyLoc, xyLoc> bounds;
  bounds.first = graph.loc(p);
  bounds.second = graph.loc(p);

  std::vector<map_position> neighbours;
  neighbours.push_back(p);
  while (neighbours.size() > 0)
  {
    map_position neighbour = neighbours.back();
    neighbours.pop_back();
    if (island_index[neighbour] == -2)
    {
      island_index[neighbour] = curr_island_index;
      bounds = graph.get_bounds_of_points(bounds, graph.loc(neighbour));
      for (const auto adj_loc_and_dist : graph.adjacent_locations_and_dists(neighbour))
      {
        neighbours.push_back(adj_loc_and_dist.first);
      }
    }
  }

  island_bounds.push_back(bounds);
}

Islands::Islands(const Graph &input_graph) : graph(input_graph)
{
  // Work out which island each point on the graph belongs to
  island_index.resize(graph.num_positions(), -2);
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    if (graph.is_obstacle(p))
      island_index[p] = -1;
  }
  int curr_island_index = 0;
  bool found_island = true;
  while (found_island)
  {
    found_island = false;
    for (map_position p = 0; p < graph.num_positions(); p++)
    {
      if (island_index[p] == -2)
      {
        found_island=true;
        create_island_from_point(p, curr_island_index);
        curr_island_index++;
      }
    }
  }
}
