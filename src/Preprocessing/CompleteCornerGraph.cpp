#include "CompleteCornerGraph.hpp"

#include "../Utility/SaveLoad.hpp"
#include "../Graph/Reachable.hpp"
#include "../Run/DirectPath.hpp"

#include <deque>

void CompleteCornerGraph::find_optimal_distances_from_corner(corner_index i, const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners)
{
  std::deque<corner_index> open;
  std::vector<bool> corner_index_is_closed(corner_vector.size(), false);

  std::vector<exact_distance> &corner_index_to_distance = pair_of_corner_indices_to_dist[i];
  corner_index_to_distance.resize(corner_vector.size(), MAX_EXACT_DISTANCE);

  open.push_back(i);
  corner_index_to_distance[i] = ZERO_EXACT_DISTANCE;
  
  corner_index curr_index;
  while (!open.empty())
  {
    curr_index = open.front();
    open.pop_front();

    if (corner_index_is_closed[curr_index])
    {
      continue;
    }
    corner_index_is_closed[curr_index] = true;

    for (corner_index neighbour_index : nearby_corners.get_nearby_corner_indices(corner_vector.get_corner(curr_index)))
    {
      exact_distance new_distance = corner_index_to_distance[curr_index] + Reachable::octile_distance(graph, corner_vector.get_corner(curr_index), corner_vector.get_corner(neighbour_index));
      assert(new_distance >= ZERO_EXACT_DISTANCE);
      if (new_distance < corner_index_to_distance[neighbour_index])
      {
        open.push_back(neighbour_index);
        corner_index_to_distance[neighbour_index] = new_distance;
        corner_index_is_closed[neighbour_index] = false;
      }
    }
  }

  corner_index_to_distance.resize(i);
}

void CompleteCornerGraph::find_optimal_first_corners_from_corner_to_corner(corner_index i, corner_index j, const CornerVector &corner_vector, const NearbyCorners &nearby_corners)
{
  if (i == j)
  {
    pair_of_corner_indices_to_first_corner[i][j] = corner_vector.size();
    return;
  }

  // It's possible to have two elements in different components, so there is no
  // way to reach one from the other
  // TODO: check more thoroughly that they really are in different components
  // and it's not just that for example the algorithm failed to find a path
  // due to a bug. I think we should probably have a component id for each
  // connected component, which would make it quick to check this.
  if (get_exact_distance_between_corner_indices(i,j) == MAX_EXACT_DISTANCE)
  {
    return;
  }

  for (corner_index first_index : nearby_corners.get_nearby_corner_indices(corner_vector.get_corner(i)))
  {
    assert(first_index != i);
    const exact_distance i_to_first_dist = get_exact_distance_between_corner_indices(i,first_index);
    const exact_distance first_to_j_dist = get_exact_distance_between_corner_indices(first_index,j);
    const exact_distance i_to_j_dist = get_exact_distance_between_corner_indices(i,j);

    assert(i_to_first_dist + first_to_j_dist >= i_to_j_dist);
    if (i_to_first_dist + first_to_j_dist == i_to_j_dist)
    {
      pair_of_corner_indices_to_first_corner[j][i] = first_index;
      return;
    }
  }

  assert(false);
}

void CompleteCornerGraph::find_optimal_first_corners_from_corner(corner_index i, const CornerVector &corner_vector, const NearbyCorners &nearby_corners)
{
  for (corner_index other_index = 0; other_index < corner_vector.size(); other_index++)
  {
    find_optimal_first_corners_from_corner_to_corner(i, other_index, corner_vector, nearby_corners);
  }
}

/*void CompleteCornerGraph::_push_corners_in_corner_graph()
{
  int num_pushed = 0;
  int num_unpushed = 0;

  // initialize replacement first corner map
  std::vector<std::vector<corner_index>> new_first_corner_map;
  new_first_corner_map.resize(corner_vector.size());
  for (corner_index i = 0; i < corner_vector.size(); i++)
    new_first_corner_map[i].resize(corner_vector.size(), corner_vector.size());

  // For every target and source index, see how far you can get using compute_octile_path
  std::vector<xyLoc> try_path;
  corner_index target_index;
  for (target_index = 0; target_index < corner_vector.size(); target_index++)
  {
    if (target_index % 100 == 0)
      std::cout << target_index << (target_index != corner_vector.size() - 1 ? ", " : "") << std::flush;
    for (corner_index source_index = 0; source_index < corner_vector.size(); source_index++)
    {
      const map_position source = corner_vector.get_corner(source_index);
      if (source_index == target_index || get_exact_distance_between_corner_indices(target_index,source_index) == MAX_EXACT_DISTANCE)
      {
        continue;
      }

      // See how far you can get
      corner_index furthest_index = source_index;
      while (furthest_index != target_index)
      {
        const corner_index next_index = pair_of_corner_indices_to_first_corner[target_index][furthest_index];
        const map_position next_corner = corner_vector.get_corner(next_index);
        try_path.clear();
        _compute_octile_path<true>(graph.loc(source), graph.loc(next_corner), try_path);
        if (graph.pos(try_path.back()) != next_corner)
        {
          break;
        }
        furthest_index = next_index;
      }
      if (furthest_index != source_index)
      {
        new_first_corner_map[target_index][source_index] = furthest_index;
        if (furthest_index == pair_of_corner_indices_to_first_corner[target_index][source_index])
          num_unpushed++;
        else
          num_pushed++;
      }
    }
  }
  if ((target_index - 1) % 100 != 0)
    std::cout << target_index - 1;
  std::cout << std::endl;

  pair_of_corner_indices_to_first_corner.swap(new_first_corner_map);
  std::cout << num_pushed << " pushed" << std::endl;
  std::cout << num_unpushed << " unpushed" << std::endl;
}*/

void CompleteCornerGraph::preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners)
{
  pair_of_corner_indices_to_dist.clear();
  pair_of_corner_indices_to_first_corner.clear();

  pair_of_corner_indices_to_dist.resize(corner_vector.size());
  pair_of_corner_indices_to_first_corner.resize(corner_vector.size());
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    pair_of_corner_indices_to_first_corner[i].resize(corner_vector.size(), corner_vector.size());
  }

  corner_index i;
  for (i = 0; i < corner_vector.size(); i++)
  { 
    if (i % 100 == 0)
      std::cout << i << ", " << std::flush;
    find_optimal_distances_from_corner(i, graph, corner_vector, nearby_corners);
  }
  std::cout << i - 1 << std::endl;

  for (i = 0; i < corner_vector.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << ", " << std::flush;
    find_optimal_first_corners_from_corner(i, corner_vector, nearby_corners);
  }
  std::cout << i - 1 << std::endl;
}

void CompleteCornerGraph::save(std::ostream &stream, const CornerVector &corner_vector) const
{
  int num_exact_distances = 0;
  int num_first_corners = 0;
  // Save distances and first corners
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    for (corner_index j = 0; j < i; j++)
    {
      exact_distance d = get_exact_distance_between_corner_indices(i,j);
      SaveLoad::save_as_binary(stream, d.num_straight);
      SaveLoad::save_as_binary(stream, d.num_diagonal);
      num_exact_distances++;
    }
    for (corner_index j = 0; j < corner_vector.size(); j++)
    {
      assert(pair_of_corner_indices_to_first_corner[i][j] != j);
      SaveLoad::save_as_binary(stream, pair_of_corner_indices_to_first_corner[i][j]);
      num_first_corners++;
    }
  }

  std::cout << num_exact_distances << " exact distances saved" << std::endl;
  std::cout << num_first_corners << " first corners saved" << std::endl;
}

void CompleteCornerGraph::load(std::istream &stream, const CornerVector &corner_vector)
{
  pair_of_corner_indices_to_dist.clear();
  pair_of_corner_indices_to_first_corner.clear();

  // Load distances and first corners
  pair_of_corner_indices_to_dist.resize(corner_vector.size());
  pair_of_corner_indices_to_first_corner.resize(corner_vector.size());
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    pair_of_corner_indices_to_dist[i].reserve(i);
    pair_of_corner_indices_to_first_corner[i].reserve(corner_vector.size());
    for (corner_index j = 0; j < i; j++)
    {
      exact_distance d;
      SaveLoad::load_as_binary(stream, d.num_straight);
      SaveLoad::load_as_binary(stream, d.num_diagonal);
      pair_of_corner_indices_to_dist[i].push_back(d);
    }
    for (corner_index j = 0; j < corner_vector.size(); j++)
    {
      corner_index first_corner;
      SaveLoad::load_as_binary(stream, first_corner);
      assert(first_corner != j);
      pair_of_corner_indices_to_first_corner[i].push_back(first_corner);
    }
  }
}

void CompleteCornerGraph::print_first(const corner_index i, const corner_index j, Printer &printer, const Graph &graph, const CornerVector &corner_vector) const
{
  assert (i != j);

  corner_index first = pair_of_corner_indices_to_first_corner[j][i];

  xyLoc a = graph.loc(corner_vector.get_corner(i));
  xyLoc b = graph.loc(corner_vector.get_corner(j));
  xyLoc o = graph.loc(corner_vector.get_corner(first));

  printer.add_char('A', a);
  printer.add_char('B', b);
  printer.add_char('O', o);
}

void CompleteCornerGraph::print_first_and_dist(const corner_index i, const corner_index j, const Graph &graph, const CornerVector &corner_vector) const
{
  assert (i != j);
  Printer printer;
  graph.print(printer);
  print_first(i,j,printer,graph,corner_vector);
  printer.print();
  std::cout << get_exact_distance_between_corner_indices(i,j) << std::endl;
}

void CompleteCornerGraph::print_all_first_and_dist(const Graph &graph, const CornerVector &corner_vector) const
{
  for (corner_index i = 0; i < corner_vector.size(); i++)
    for (corner_index j = 0; j < corner_vector.size(); j++)
    {
      if (i == j)
        continue;

      print_first_and_dist(i,j,graph,corner_vector);
    }
}

void CompleteCornerGraph::push_first_corners(const Graph &graph, const CornerVector &corner_vector)
{
  const auto &first_corners = pair_of_corner_indices_to_first_corner;
  std::vector<std::vector<corner_index>> new_first_corners;
  new_first_corners.resize(corner_vector.size());
  for (auto &row : new_first_corners)
    row.resize(corner_vector.size());

  corner_index i;
  for (i = 0; i < corner_vector.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << ", " << std::flush;
    const xyLoc i_loc = graph.loc(corner_vector.get_corner(i));
    for (corner_index j = 0; j < corner_vector.size(); j++)
    {
      if (i == j)
        new_first_corners[j][i] = corner_vector.size();

      // Push the corner i, j
      corner_index new_first_corner = first_corners[j][i];
      while (true)
      {
        if (new_first_corner == j)
          break;
        const corner_index next_first_corner = first_corners[j][new_first_corner];
        const xyLoc next_first_corner_loc = graph.loc(corner_vector.get_corner(next_first_corner));
        std::vector<xyLoc> path;
        Running::compute_octile_path<true>(i_loc, next_first_corner_loc, path, graph);
        const bool can_get_to_next_first_corner = (!path.empty() && next_first_corner_loc == path.back());
        if (can_get_to_next_first_corner)
          new_first_corner = next_first_corner;
        else
          break;
      }
      assert(i < corner_vector.size());
      assert(j < corner_vector.size());
      assert(new_first_corners.size() == corner_vector.size());
      assert(new_first_corners[j].size() == corner_vector.size());
      new_first_corners[j][i] = new_first_corner;
    }
  }
  std::cout << i - 1<< std::endl;
}

