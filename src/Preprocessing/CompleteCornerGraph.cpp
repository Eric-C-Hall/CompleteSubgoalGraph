#include "CompleteCornerGraph.hpp"

#include "../Utility/SaveLoad.hpp"

#include <deque>

void CompleteCornerGraph::_find_optimal_distances_from_corner(corner_index i, const CornerVector &corner_vector)
{
  std::deque<corner_index> open;
  std::vector<bool> corner_index_is_closed(corner_vector.size(), false);

  std::vector<exact_distance> &corner_index_to_distance = _pair_of_corner_indices_to_dist[i];
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
      exact_distance new_distance = corner_index_to_distance[curr_index] + graph.octile_distance(corner_vector.get_corner(curr_index), corner_vector.get_corner(neighbour_index));
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

void CompleteCornerGraph::_find_optimal_first_corners_from_corner_to_corner(corner_index i, corner_index j, const CornerVector &corner_vector)
{
  if (i == j)
  {
    _pair_of_corner_indices_to_first_corner[i][j] = corner_vector.size();
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
      _pair_of_corner_indices_to_first_corner[j][i] = first_index;
      return;
    }
  }

  assert(false);
}

void CompleteCornerGraph::_find_optimal_first_corners_from_corner(corner_index i, const CornerVector &corner_vector)
{
  for (corner_index other_index = 0; other_index < corner_vector.size(); other_index++)
  {
    _find_optimal_first_corners_from_corner_to_corner(i, other_index);
  }
}

void CompleteCornerGraph::_find_complete_corner_graph(const CornerVector &corner_vector)
{
  _pair_of_corner_indices_to_dist.resize(corner_vector.size());
  _pair_of_corner_indices_to_first_corner.resize(corner_vector.size());
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    _pair_of_corner_indices_to_first_corner[i].resize(corner_vector.size(), corner_vector.size());
  }

  corner_index i;
  for (i = 0; i < corner_vector.size(); i++)
  {

    if (i % 100 == 0)
      std::cout << i << (i != corner_vector.size() - 1 ? ", " : "") << std::flush;
    _find_optimal_distances_from_corner(i);
  }
  if ((i - 1) % 100 != 0)
    std::cout << i - 1;
  std::cout << std::endl;

  for (i = 0; i < corner_vector.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << (i != corner_vector.size() - 1 ? ", " : "") << std::flush;
    _find_optimal_first_corners_from_corner(i);
  }
  if ((i - 1) % 100 != 0)
    std::cout << i - 1;
  std::cout << std::endl;
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
        const corner_index next_index = _pair_of_corner_indices_to_first_corner[target_index][furthest_index];
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
        if (furthest_index == _pair_of_corner_indices_to_first_corner[target_index][source_index])
          num_unpushed++;
        else
          num_pushed++;
      }
    }
  }
  if ((target_index - 1) % 100 != 0)
    std::cout << target_index - 1;
  std::cout << std::endl;

  _pair_of_corner_indices_to_first_corner.swap(new_first_corner_map);
  std::cout << num_pushed << " pushed" << std::endl;
  std::cout << num_unpushed << " unpushed" << std::endl;
}*/

void CompleteCornerGraph::preprocess()
{
  
}

void CompleteCornerGraph::save(std::ostream &stream, const CornerVector &corner_vector) const;
{
  int num_exact_distances = 0;
  int num_first_corners = 0;
  // Save _pair_of_corner_indices_to_first_corner and _pair_of_corner_indices_to_dist
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    for (corner_index j = 0; j <= i; j++)
    {
      exact_distance d = _pair_of_corner_indices_to_dist[i][j];
      SaveLoad::save_uint16_t_as_binary(stream, d.num_straight);
      SaveLoad::save_uint16_t_as_binary(stream, d.num_diagonal);
      num_exact_distances++;
    }
    for (corner_index j = 0; j < corner_vector.size(); j++)
    {
      assert(_pair_of_corner_indices_to_first_corner[i][j] != j);
      SaveLoad::save_uint16_t_as_binary(stream, _pair_of_corner_indices_to_first_corner[i][j]);
      num_first_corners++;
    }
  }

  std::cout << num_exact_distances << " exact distances saved" << std::endl;
  std::cout << num_first_corners << " first corners saved" << std::endl;
}

void CompleteCornerGraph::load(std::istream &stream, const CornerVector &corner_vector)
{
  // Load _pair_of_corner_indices_to_dist and _pair_of_corner_indices_to_first_corner
  _pair_of_corner_indices_to_dist.resize(corner_vector.size());
  _pair_of_corner_indices_to_first_corner.resize(corner_vector.size());
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    _pair_of_corner_indices_to_dist[i].reserve(i+1);
    _pair_of_corner_indices_to_first_corner[i].reserve(corner_vector.size());
    for (corner_index j = 0; j <= i; j++)
    {
      int num_straight = SaveLoad::load_uint16_t_as_binary(stream);
      int num_diagonal = SaveLoad::load_uint16_t_as_binary(stream);
      exact_distance d(num_straight, num_diagonal);
      _pair_of_corner_indices_to_dist[i].push_back(d);
    }
    for (corner_index j = 0; j < corner_vector.size(); j++)
    {
      corner_index first_corner = SaveLoad::load_uint16_t_as_binary(stream);
      assert(first_corner != j);
      _pair_of_corner_indices_to_first_corner[i].push_back(first_corner);
    }
  }
}
