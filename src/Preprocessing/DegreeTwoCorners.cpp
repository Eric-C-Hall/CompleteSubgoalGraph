#include "DegreeTwoCorners.hpp"

#include <deque>
#include <algorithm>
#include <numeric>

bool DegreeTwoCorners::path_through_index_optimal(corner_index a, corner_index b, corner_index i, const CompleteCornerGraph &complete_corner_graph) const
{
  exact_distance ab_dist = complete_corner_graph.get_exact_distance_between_corner_indices(a,b);
  exact_distance ai_dist = complete_corner_graph.get_exact_distance_between_corner_indices(a,i);
  exact_distance ib_dist = complete_corner_graph.get_exact_distance_between_corner_indices(i,b);
  return ab_dist == ai_dist + ib_dist;
}

bool DegreeTwoCorners::no_path_through_other_indices(corner_index a, corner_index b, corner_index i, const CornerVector &corner_vector, const NearbyCorners &nearby_corners, const CompleteCornerGraph &complete_corner_graph) const
{
  assert (a != i);
  assert (b != i);
  assert (a != b);

  std::deque<corner_index> open;
  open.push_back(a);

  std::vector<corner_index> closed;

  while (!open.empty())
  {
    corner_index curr_index = open.front();
    open.pop_front();

    bool is_closed = std::find(closed.begin(), closed.end(), curr_index) != closed.end();
    if (is_closed)
      continue;
    closed.push_back(curr_index);

    for (corner_index next_index : nearby_corners.get_nearby_corner_indices(corner_vector.get_corner(curr_index)))
    {
      if (next_index == b)
        return false;

      if (!path_through_index_optimal(curr_index,b,next_index,complete_corner_graph))
        continue;

      if (!is_degree_two[next_index])
        continue;

      if (next_index == i)
        continue;

      open.push_back(next_index);
    }
  }
  return true;
}

// A corner can be assumed to be degree one if for any two adjacent corners this corner is optimal for, there exists an optimal path between them that does not pass through this corner or any degree one corner
bool DegreeTwoCorners::compute_is_degree_one(corner_index i, const CornerVector &corner_vector, const NearbyCorners &nearby_corners, const CompleteCornerGraph &complete_corner_graph) const
{
  auto adjacent_indices = nearby_corners.get_nearby_corner_indices(corner_vector.get_corner(i));
  for (corner_index a : adjacent_indices)
  {
    for (corner_index b : adjacent_indices)
    {
      if (path_through_index_optimal(a,b,i,complete_corner_graph))
        if (no_path_through_other_indices(a, b, i,corner_vector,nearby_corners,complete_corner_graph))
          return false;
    }
  }
  return true;
}

void DegreeTwoCorners::preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners, const CompleteCornerGraph &complete_corner_graph)
{
  is_degree_two.clear();
  is_degree_two.resize(corner_vector.size(), true);

  std::vector<corner_index> curr_check_indices(corner_vector.size());
  std::iota(curr_check_indices.begin(), curr_check_indices.end(), 0);

  std::vector<corner_index> next_check_indices;

  // For every index
  int num_indices_added = 0;
  bool index_added_this_iteration = true;
  while (index_added_this_iteration)
  {
    index_added_this_iteration = false;
    for (corner_index i : curr_check_indices)
    {
      // Check if every pair of adjacent corners with optimal path through i also has optimal path through degree 2 indices not including this index
      bool is_degree_one = compute_is_degree_one(i, corner_vector, nearby_corners, complete_corner_graph);

      // If so, this corner has degree one. Else, add to indices to check next time.
      if (is_degree_one)
      {
        is_degree_two[i] = false;
        index_added_this_iteration = true;
        num_indices_added++;
      }
      else
      {
        next_check_indices.push_back(i);
      }
    }
    curr_check_indices.clear();
    curr_check_indices.swap(next_check_indices);
  }

  std::cout << num_indices_added << "/" << corner_vector.size() << " indices are degree one" << std::endl;
}

void DegreeTwoCorners::print(Printer &printer, const Graph &graph, const CornerVector &corner_vector) const
{
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    if (is_degree_two[i])
    {
      printer.add_char('2', graph.loc(corner_vector.get_corner(i)));
    }
  }
}
