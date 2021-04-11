#include "Preprocess.hpp"

#include <cassert>
#include <iostream>
#include <fstream>
#include <deque>

#include "../Graph/ExactDistance.hpp"

#include "../Utility/Timer.h"

PreprocessingData::PreprocessingData(const Graph &input_graph) : graph(input_graph)
{
}



void PreprocessingData::_add_if_corner(map_position p)
{
  if (graph.is_obstacle(p))
  {
    return;
  }

  if (graph.is_obstacle(graph.left(p)))
  {
    if (graph.is_obstacle(graph.down(p)) || graph.is_obstacle(graph.up(p)))
    {
      return;
    }
    if (!graph.is_obstacle(graph.left(graph.down(p))) || !graph.is_obstacle(graph.left(graph.up(p))))
    {
      _corners.push_back(p);
      return;
    }
  }

  if (graph.is_obstacle(graph.right(p)))
  {
    if (graph.is_obstacle(graph.down(p)) || graph.is_obstacle(graph.up(p)))
    {
      return;
    }
    if (!graph.is_obstacle(graph.right(graph.down(p))) || !graph.is_obstacle(graph.right(graph.up(p))))
    {
      _corners.push_back(p);
      return;
    }
  }

  if (graph.is_obstacle(graph.up(p)))
  {
    if (graph.is_obstacle(graph.left(p)) || graph.is_obstacle(graph.right(p)))
    {
      return;
    }
    if (!graph.is_obstacle(graph.up(graph.left(p))) || !graph.is_obstacle(graph.up(graph.right(p))))
    {
      _corners.push_back(p);
      return;
    }
  }

  if (graph.is_obstacle(graph.down(p)))
  {
    if (graph.is_obstacle(graph.left(p)) || graph.is_obstacle(graph.right(p)))
    {
      return;
    }
    if (!graph.is_obstacle(graph.down(graph.left(p))) || !graph.is_obstacle(graph.down(graph.right(p))))
    {
      _corners.push_back(p);
      return;
    }
  }
}

void PreprocessingData::_compute_corners()
{
  _corners.clear();
  for (unsigned int x=1 ; x<graph.get_width()-1 ; x++)
  {
    for (unsigned int y=1 ; y<graph.get_height()-1 ; y++)
    {
      _add_if_corner(graph.pos(x,y));
    }
  }

  _corner_to_corner_index.clear();
  _corner_to_corner_index.resize(graph.num_positions(), _corners.size());  
  for (corner_index i = 0; i < _corners.size(); i++)
  {
    _corner_to_corner_index[_corners[i]] = i;
  }

  std::cout << "Number of corners: " << _corners.size() << std::endl;
}

// Maybe using exact distances here is slow?
void PreprocessingData::_find_points_near_corner(corner_index i)
{
  if (graph.is_obstacle(_corners[i]))
  {
    return;
  }

  const corner_index NO_CORNER = _corners.size();

  std::deque<map_position> buckets[3];
  std::vector<exact_distance> distances(graph.num_positions(), MAX_EXACT_DISTANCE);
  std::vector<bool> has_found_corner(graph.num_positions(), false);
  std::vector<bool> is_closed(graph.num_positions(), false);

  // This decides whether a new position should go into the second or third bucket. It is the smallest value that should go in the third bucket.  
  exact_distance bucket_decider = 2 * STRAIGHT_EXACT_DISTANCE;

  buckets[0].push_back(_corners[i]);
  distances[_corners[i]] = ZERO_EXACT_DISTANCE;
  int last_bucket_containing_point_that_hasnt_found_corner = 0;
  
  while (last_bucket_containing_point_that_hasnt_found_corner > -1)
  {
    while (!buckets[0].empty())
    {
      const map_position curr_pos = buckets[0].front();
      buckets[0].pop_front();

      if (is_closed[curr_pos])
      {
        continue;
      }
      is_closed[curr_pos] = true;

      const corner_index curr_corner_index = _corner_to_corner_index[curr_pos];
      if (curr_corner_index != i)
      {
        if (!has_found_corner[curr_pos])
        _point_to_nearby_corner_indices[curr_pos].push_back(i);

        if (curr_corner_index != NO_CORNER)
          has_found_corner[curr_pos] = true;
      }

      for (auto loc_and_dist : graph.adjacent_locations_and_dists(curr_pos))
      {
        map_position other_pos = loc_and_dist.first;

        if (graph.is_obstacle(other_pos))
        {
          continue;
        }

        const exact_distance other_distance = distances[curr_pos] + loc_and_dist.second;
        if (distances[other_pos] < other_distance)
        {
          continue;
        }
        else if (distances[other_pos] == other_distance && (!has_found_corner[curr_pos] || has_found_corner[other_pos]))
        {
          continue;
        }

        distances[other_pos] = other_distance;
        has_found_corner[other_pos] = has_found_corner[curr_pos];

        if (other_distance < bucket_decider)
        {
          buckets[1].push_back(other_pos);
          if (!has_found_corner[other_pos])
          {
            last_bucket_containing_point_that_hasnt_found_corner = std::max(1, last_bucket_containing_point_that_hasnt_found_corner);
          }
        }
        else
        {
          buckets[2].push_back(other_pos);
          if (!has_found_corner[other_pos])
          {
            last_bucket_containing_point_that_hasnt_found_corner = 2;
          }
        }
      }
    }
    buckets[0].swap(buckets[1]);
    buckets[1].swap(buckets[2]);
    bucket_decider += STRAIGHT_EXACT_DISTANCE;
    last_bucket_containing_point_that_hasnt_found_corner--;
  }
}

void PreprocessingData::_find_nearby_corners()
{
  _point_to_nearby_corner_indices.clear();
  _point_to_nearby_corner_indices.resize(graph.get_width() * graph.get_height());

  // From every corner, find nearby points, and remember that this corner is near these points.
  for (corner_index i = 0; i < _corners.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << (i != _corners.size() - 1 ? ", " : "") << std::flush;
    _find_points_near_corner(i);
  }
  std::cout << "\n";
}

void PreprocessingData::_find_optimal_distances_from_corner(corner_index i)
{
  std::deque<corner_index> open;
  std::vector<bool> corner_index_is_closed(_corners.size(), false);
  std::vector<exact_distance> corner_index_to_distance(_corners.size(), MAX_EXACT_DISTANCE);

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

    _pair_of_corner_indices_to_dist[i][curr_index] = corner_index_to_distance[curr_index];

    for (corner_index neighbour_index : _point_to_nearby_corner_indices[_corners[curr_index]])
    {
      exact_distance new_distance = corner_index_to_distance[curr_index] + graph.octile_distance(_corners[curr_index], _corners[neighbour_index]);
      if (new_distance < corner_index_to_distance[neighbour_index])
      {
        open.push_back(neighbour_index);
        corner_index_to_distance[neighbour_index] = new_distance;
        corner_index_is_closed[neighbour_index] = false;
      }
    }
  }
}

void PreprocessingData::_find_optimal_first_corners_from_corner(corner_index i)
{
  const auto nearby_indices = _point_to_nearby_corner_indices[_corners[i]];
  for (corner_index other_index = 0; other_index < _corners.size(); other_index++)
  {
    for (corner_index first_index : nearby_indices)
    {
      assert(first_index != i);
      if (_pair_of_corner_indices_to_dist[i][first_index] + _pair_of_corner_indices_to_dist[first_index][other_index] == _pair_of_corner_indices_to_dist[i][other_index])
      {
        _pair_of_corner_indices_to_first_corner[i][other_index] = first_index;
        break;
      }
    }
  }
}

void PreprocessingData::_find_complete_corner_graph()
{
  _pair_of_corner_indices_to_dist.resize(_corners.size());
  _pair_of_corner_indices_to_first_corner.resize(_corners.size());
  for (corner_index i = 0; i < _corners.size(); i++)
  {
    _pair_of_corner_indices_to_dist[i].resize(_corners.size());
    _pair_of_corner_indices_to_first_corner[i].resize(_corners.size());
  }

  for (corner_index i = 0; i < _corners.size(); i++)
  {

    if (i % 100 == 0)
      std::cout << i << (i != _corners.size() - 1 ? ", " : "") << std::flush;
    _find_optimal_distances_from_corner(i);
  }
  std::cout << std::endl;

  for (corner_index i = 0; i < _corners.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << (i != _corners.size() - 1 ? ", " : "") << std::flush;
    _find_optimal_first_corners_from_corner(i);
  }
  std::cout << std::endl;
}

void PreprocessingData::_save(std::ostream & stream) const
{
  stream << _corners.size() << "\n";

  for (unsigned int corner : _corners)
  {
    stream << corner << " ";
  }
  stream << "\n\n";

  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    for (unsigned int corner : _point_to_nearby_corner_indices[p])
    {
      stream << corner << " ";
    }
    stream << _corners.size() << "\n";
  }

  stream << "\n";

  for (corner_index i = 0; i < _corners.size(); i++)
  {
    for (corner_index j = 0; j < _corners.size(); j++)
    {
      exact_distance d = _pair_of_corner_indices_to_dist[i][j];
      stream << d.num_straight << " " << d.num_diagonal << " ";
      stream << _pair_of_corner_indices_to_first_corner[i][j] << " ";
    }
    stream << "\n";
  }
}

void PreprocessingData::save(const std::string &filename) const
{
  std::cout << "Saving preprocessed data" << std::endl;
  std::ofstream file(filename, std::ifstream::out);
  if (file.fail())
  {
    std::cerr << "Attempt to save \"" << filename << "\"" << std::endl;
    throw std::runtime_error("Error saving preprocessed data file.");
  }
  _save(file);
  file.close();
  std::cout << "Saving complete" << std::endl;
}

void PreprocessingData::_load(std::istream &stream)
{
  _corners.clear();
  _corner_to_corner_index.clear();
  _point_to_nearby_corner_indices.clear();
  _pair_of_corner_indices_to_dist.clear();
  _pair_of_corner_indices_to_first_corner.clear();

  unsigned int num_corners;
  stream >> num_corners;

  _corners.reserve(num_corners);
  for (unsigned int i = 0; i < num_corners; i++)
  {
    map_position corner_pos;
    stream >> corner_pos;
    _corners.push_back(corner_pos);
  }

  _point_to_nearby_corner_indices.resize(graph.num_positions());
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    map_position corner_index;
    stream >> corner_index;
    while (corner_index != num_corners)
    {
      _point_to_nearby_corner_indices[p].push_back(corner_index);
      stream >> corner_index;
    }
  }

  _pair_of_corner_indices_to_dist.resize(_corners.size());
  _pair_of_corner_indices_to_first_corner.resize(_corners.size());
  for (corner_index i = 0; i < _corners.size(); i++)
  {
    _pair_of_corner_indices_to_dist[i].reserve(_corners.size());
    _pair_of_corner_indices_to_first_corner[i].reserve(_corners.size());
    for (corner_index j = 0; j < _corners.size(); j++)
    {
      int num_straight;
      int num_diagonal;
      stream >> num_straight >> num_diagonal;
      exact_distance d(num_straight, num_diagonal);
      _pair_of_corner_indices_to_dist[i].push_back(d);

      corner_index first_corner;
      stream >> first_corner;
      _pair_of_corner_indices_to_first_corner[i].push_back(first_corner);
    }
  }
}

void PreprocessingData::load(const std::string &filename)
{
  std::ifstream file(filename, std::ifstream::in);
  if (file.fail())
  {
    std::cerr << "Attempt to open \"" << filename << "\"" << std::endl;
    throw std::runtime_error("Error opening preprocessed data file.");
  }
  _load(file);
}

void PreprocessingData::preprocess()
{
  Timer t;

  std::cout << "Computing corners" << std::endl;
  t.StartTimer();
  _compute_corners();
  t.EndTimer();
  std::cout << "Corners computed in " << t.GetElapsedTime() << std::endl;
  std::cout << "Finding nearby corners" << std::endl;
  t.StartTimer();
  _find_nearby_corners();
  t.EndTimer();
  std::cout << "Nearby corners found in " << t.GetElapsedTime() << std::endl;
  std::cout << "Finding complete corner graph" << std::endl;
  t.StartTimer();
  _find_complete_corner_graph(); 
  t.EndTimer(); 
  std::cout << "Complete corner graph found in " << t.GetElapsedTime() << std::endl;
  std::cout << "Preprocessing complete" << std::endl;
}

// Doesn't add start to path, but does add end.
template<bool x_diff_greater_than_y_diff, bool test_valid>
void PreprocessingData::_compute_diagonal_then_straight_path(xyLoc start, int16_t num_diagonal, int16_t num_straight, int16_t x_step, int16_t y_step, std::vector<xyLoc> &path) const
{
  while(num_diagonal > 0)
  {
    start.x += x_step;
    start.y += y_step;
    if (test_valid)
      if (graph.is_obstacle(graph.pos(start.x, start.y)))
        return;
    path.push_back(start);
    num_diagonal--;
  }
  while(num_straight > 0)
  {
    if (x_diff_greater_than_y_diff)
      start.x += x_step;
    else
      start.y += y_step;
    if (test_valid)
      if (graph.is_obstacle(graph.pos(start.x, start.y)))
        return;
    path.push_back(start);
    num_straight--;
  }
}

// Doesn't add start to path, but does add end.
template<bool test_valid>
void PreprocessingData::_compute_octile_path(xyLoc start, xyLoc end, std::vector<xyLoc> &path) const
{
  int16_t x_diff = std::abs(start.x - end.x);
  int16_t y_diff = std::abs(start.y - end.y);
  int16_t x_step = (start.x > end.x ? -1 : 1);
  int16_t y_step = (start.y > end.y ? -1 : 1);

  if (x_diff > y_diff)
    _compute_diagonal_then_straight_path<true, test_valid>(start, y_diff, x_diff - y_diff, x_step, y_step, path);
  else
    _compute_diagonal_then_straight_path<false, test_valid>(start, x_diff, y_diff - x_diff, x_step, y_step, path);
}

bool PreprocessingData::get_path(map_position start, map_position goal, std::vector<xyLoc> &path) const
{
  path.push_back(graph.loc(start));
  _compute_octile_path<true>(graph.loc(start), graph.loc(goal), path);
  if (path.back() == graph.loc(goal))
  {
    return true;
  }
  path.clear();

  exact_distance shortest_distance = MAX_EXACT_DISTANCE;
  corner_index best_start_index;
  corner_index best_end_index;

  //std::cout << _point_to_nearby_corner_indices[start].size() << " " << _point_to_nearby_corner_indices[goal].size() << " " << _point_to_nearby_corner_indices[start].size() * _point_to_nearby_corner_indices[goal].size() << std::endl;

  for (corner_index i : _point_to_nearby_corner_indices[start])
  {
    exact_distance i_dist = graph.octile_distance(start, _corners[i]);
    for (corner_index j : _point_to_nearby_corner_indices[goal])
    {
      exact_distance j_dist = graph.octile_distance(goal, _corners[j]);
      exact_distance current_dist = i_dist + _pair_of_corner_indices_to_dist[i][j] + j_dist;
      if (current_dist < shortest_distance)
      {
        shortest_distance = current_dist;
        best_start_index = i;
        best_end_index = j;
      }
    }
  }

  xyLoc a = graph.loc(start);
  path.push_back(a);
  
  corner_index start_index = best_start_index;
  corner_index end_index = best_end_index;

  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
  xyLoc b = graph.loc(_corners[start_index]);
  #pragma GCC diagnostic pop

  _compute_octile_path<false>(a, b, path);
  while (start_index != end_index)
  {
    a = b;
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
    corner_index new_index = _pair_of_corner_indices_to_first_corner[start_index][end_index];
    #pragma GCC diagnostic pop
    assert(new_index != start_index);
    start_index = new_index;
    b = graph.loc(_corners[start_index]);
    _compute_octile_path<false>(a, b, path);
  }
  _compute_octile_path<false>(b, graph.loc(goal), path);

  return true;
}

std::vector<map_position> PreprocessingData::get_nearby_corners(map_position p) const
{
  std::vector<map_position> return_value;
  auto nearby_indices = _point_to_nearby_corner_indices[p];
  return_value.reserve(nearby_indices.size());

  for (corner_index i : nearby_indices)
  {
    return_value.push_back(_corners[i]);
  }

  return return_value;
}
