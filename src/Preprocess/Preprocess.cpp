#include "Preprocess.hpp"

#include <cassert>
#include <iostream>
#include <fstream>
#include <deque>
#include <tuple>
#include <algorithm>

#include "../Graph/ExactDistance.hpp"
#include "../Graph/Directions.hpp"

#include "../Utility/Timer.h"
#include "../Utility/Stats.hpp"

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

  std::cout << "Number of corners: " << _corners.size() << std::endl;
}

// Hitting an obstacle decreases num_step_bound, to ensure that all points are safe-reachable rather than just octile-reachable
template<Direction dir, Direction step_dir>
void PreprocessingData::_find_points_near_corner_straight(map_position initial_pos, corner_index i, int num_step_bound)
{
  map_position curr_pos = initial_pos;
  int num_steps = 0;

  while (!graph.is_obstacle(curr_pos) && num_steps < num_step_bound)
  {
    // The first iteration is done twice with two different step directions.
    // So the same point may be reached multiple times, so only add this corner index the first time
    if (_point_to_nearby_corner_indices[curr_pos].empty() || _point_to_nearby_corner_indices[curr_pos].back() != i)
      _point_to_nearby_corner_indices[curr_pos].push_back(i);

    moving_direction(dir, curr_pos, graph);
    num_steps++;
  }

  if (num_steps != 0)
  {
    moving_direction(step_dir, initial_pos, graph);
    _find_points_near_corner_straight<dir, step_dir>(initial_pos, i, num_steps);
  }
}

void PreprocessingData::_find_points_near_corner(corner_index i)
{
  assert (!graph.is_obstacle(_corners[i]));

  map_position c = _corners[i];

  _find_points_near_corner_straight<Dir_N, Dir_NW>(c, i);
  _find_points_near_corner_straight<Dir_N, Dir_NE>(c, i);
  _find_points_near_corner_straight<Dir_E, Dir_NE>(c, i);
  _find_points_near_corner_straight<Dir_E, Dir_SE>(c, i);
  _find_points_near_corner_straight<Dir_S, Dir_SE>(c, i);
  _find_points_near_corner_straight<Dir_S, Dir_SW>(c, i);
  _find_points_near_corner_straight<Dir_W, Dir_SW>(c, i);
  _find_points_near_corner_straight<Dir_W, Dir_NW>(c, i);

  // Due to the way the nearby point finding algorithm works, the corner will be considered near itself.
  // This isn't what we want so we reverse this effect
  assert(_point_to_nearby_corner_indices[c].back() == i);
  _point_to_nearby_corner_indices[c].pop_back();
  assert(_point_to_nearby_corner_indices[c].empty() || _point_to_nearby_corner_indices[c].back() != i);
}

void PreprocessingData::_find_nearby_corners()
{
  _point_to_nearby_corner_indices.clear();
  _point_to_nearby_corner_indices.resize(graph.get_width() * graph.get_height());

  // From every corner, find nearby points, and remember that this corner is near these points.
  corner_index i;
  for (i = 0; i < _corners.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << (i != _corners.size() - 1 ? ", " : "") << std::flush;
    _find_points_near_corner(i);
  }
  if ((i - 1) % 100 != 0)
    std::cout << i - 1;
  std::cout << "\n";
}

void PreprocessingData::_find_optimal_distances_from_corner(corner_index i)
{
  std::deque<corner_index> open;
  std::vector<bool> corner_index_is_closed(_corners.size(), false);

  std::vector<exact_distance> &corner_index_to_distance = _pair_of_corner_indices_to_dist[i];
  corner_index_to_distance.resize(_corners.size(), MAX_EXACT_DISTANCE);

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

    for (corner_index neighbour_index : _point_to_nearby_corner_indices[_corners[curr_index]])
    {
      exact_distance new_distance = corner_index_to_distance[curr_index] + graph.octile_distance(_corners[curr_index], _corners[neighbour_index]);
      assert(new_distance >= ZERO_EXACT_DISTANCE);
      if (new_distance < corner_index_to_distance[neighbour_index])
      {
        open.push_back(neighbour_index);
        corner_index_to_distance[neighbour_index] = new_distance;
        corner_index_is_closed[neighbour_index] = false;
      }
    }
  }
}

void PreprocessingData::_find_optimal_first_corners_from_corner_to_corner(corner_index i, corner_index j)
{
  if (i == j)
  {
    _pair_of_corner_indices_to_first_corner[i][j] = _corners.size();
    return;
  }

  // It's possible to have two elements in different components, so there is no
  // way to reach one from the other
  // TODO: check more thoroughly that they really are in different components
  // and it's not just that for example the algorithm failed to find a path
  // due to a bug. I think we should probably have a component id for each
  // connected component, which would make it quick to check this.
  if (_pair_of_corner_indices_to_dist[i][j] == MAX_EXACT_DISTANCE)
  {
    return;
  }

  for (corner_index first_index : _point_to_nearby_corner_indices[_corners[i]])
  {
    assert(first_index != i);
    const exact_distance i_to_first_dist = _pair_of_corner_indices_to_dist[i][first_index];
    const exact_distance first_to_j_dist = _pair_of_corner_indices_to_dist[first_index][j];
    const exact_distance i_to_j_dist = _pair_of_corner_indices_to_dist[i][j];

    assert(i_to_first_dist + first_to_j_dist >= i_to_j_dist);
    if (i_to_first_dist + first_to_j_dist == i_to_j_dist)
    {
      _pair_of_corner_indices_to_first_corner[j][i] = first_index;
      return;
    }
  }

  assert(false);
}

void PreprocessingData::_find_optimal_first_corners_from_corner(corner_index i)
{
  for (corner_index other_index = 0; other_index < _corners.size(); other_index++)
  {
    _find_optimal_first_corners_from_corner_to_corner(i, other_index);
  }
}

void PreprocessingData::_find_complete_corner_graph()
{
  _pair_of_corner_indices_to_dist.resize(_corners.size());
  _pair_of_corner_indices_to_first_corner.resize(_corners.size());
  for (corner_index i = 0; i < _corners.size(); i++)
  {
    _pair_of_corner_indices_to_first_corner[i].resize(_corners.size(), _corners.size());
  }

  corner_index i;
  for (i = 0; i < _corners.size(); i++)
  {

    if (i % 100 == 0)
      std::cout << i << (i != _corners.size() - 1 ? ", " : "") << std::flush;
    _find_optimal_distances_from_corner(i);
  }
  if ((i - 1) % 100 != 0)
    std::cout << i - 1;
  std::cout << std::endl;

  for (i = 0; i < _corners.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << (i != _corners.size() - 1 ? ", " : "") << std::flush;
    _find_optimal_first_corners_from_corner(i);
  }
  if ((i - 1) % 100 != 0)
    std::cout << i - 1;
  std::cout << std::endl;
}

void PreprocessingData::_push_corners_in_corner_graph()
{
  int num_pushed = 0;
  int num_unpushed = 0;

  // initialize replacement first corner map
  std::vector<std::vector<corner_index>> new_first_corner_map;
  new_first_corner_map.resize(_corners.size());
  for (corner_index i = 0; i < _corners.size(); i++)
    new_first_corner_map[i].resize(_corners.size(), _corners.size());

  // For every target and source index, see how far you can get using compute_octile_path
  std::vector<xyLoc> try_path;
  corner_index target_index;
  for (target_index = 0; target_index < _corners.size(); target_index++)
  {
    if (target_index % 100 == 0)
      std::cout << target_index << (target_index != _corners.size() - 1 ? ", " : "") << std::flush;
    for (corner_index source_index = 0; source_index < _corners.size(); source_index++)
    {
      const map_position source = _corners[source_index];
      if (source_index == target_index || _pair_of_corner_indices_to_dist[target_index][source_index] == MAX_EXACT_DISTANCE)
      {
        continue;
      }

      // See how far you can get
      corner_index furthest_index = source_index;
      while (furthest_index != target_index)
      {
        const corner_index next_index = _pair_of_corner_indices_to_first_corner[target_index][furthest_index];
        const map_position next_corner = _corners[next_index];
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
      assert(_pair_of_corner_indices_to_first_corner[i][j] != j);
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
      assert(first_corner != j);
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

void PreprocessingData::_output_debug_stats() const
{
  std::cout << "Outputting debug stats" << std::endl;

  int num_non_obstacles = 0;
  for (map_position p = 0; p < graph.num_positions(); p++)
    if (!graph.is_obstacle(p))
      num_non_obstacles++;

  std::cout << std::endl;
  std::cout << "Width: " << graph.get_width() << std::endl;
  std::cout << "Height: " << graph.get_height() << std::endl;
  std::cout << "Num squares: " << graph.get_width() * graph.get_height() << std::endl;
  std::cout << "Non-obstacles: " << num_non_obstacles << std::endl;

  std::vector<unsigned int> num_nearby;
  num_nearby.reserve(graph.num_positions()); 
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    if (!graph.is_obstacle(p))
      num_nearby.push_back(_point_to_nearby_corner_indices[p].size());
  }

  auto stats = get_stats(num_nearby);
  
  std::cout << std::endl;
  std::cout << "The following stats only include non-obstacles" << std::endl;
  std::cout << "Min num nearby " << std::get<0>(stats) << std::endl;
  std::cout << "Qt1 num nearby " << std::get<5>(stats) << std::endl;
  std::cout << "Median num nearby " << std::get<3>(stats) << std::endl;
  std::cout << "Qt2 num nearby " << std::get<6>(stats) << std::endl;
  std::cout << "Max num nearby " << std::get<1>(stats) << std::endl;
  std::cout << "Mean num nearby " << std::get<2>(stats) << std::endl;
  std::cout << "Mode num nearby " << std::get<4>(stats) << std::endl;

  // Warn user about positions without nearby corners
  if (std::get<0>(stats) == 0)
  {
    std::cout << "-------------------------------------------------------------------------" << std::endl;
    std::cout << "WARNING: THERE SHOULDN'T BE NON-OBSTACLE POSITIONS WITHOUT NEARBY CORNERS" << std::endl;
    std::cout << std::endl;
    std::cout << "The following positions have no nearby corners:" << std::endl;
    std::cout << std::endl;
    
    int num_printed = 0;
    int num_unprinted = 0;
    
    for (map_position p = 0; p < graph.num_positions(); p++)
    {
      if (!graph.is_obstacle(p) && _point_to_nearby_corner_indices[p].size() == 0)
      {
        if (num_printed < 10)
        {
          std::cout << p << ": (" << graph.x(p) << ", " << graph.y(p) << ")" << std::endl;
          num_printed++;
        }
        else
        {
          num_unprinted++;
        }
      }
    }
    if (num_unprinted > 0)
    {
      std::cout << "..." << std::endl;
      std::cout << "And " << num_unprinted << " more" << std::endl;
    }
    std::cout << "-------------------------------------------------------------------------" << std::endl;
  }

}


// Assume that pos and corner are safe-reachable from each other.
//
// Here are some examples of when the corner is useful:
//
//--|||||||/-------
//---|||||/--------
//----|||/---------
//-----|/----------
//-----c###########
//-----|\----------
//----|||\---------
//---|||||\--------
//--|||||||\-------
//
//
//
//--||||----------
//---|||----------
//----||----------
//-----|##########
//-----c##########
//------\---------
//------|\--------
//------||\-------
//------|||\------
//
//
//
//-\|||||||/-------
//--\|||||/--------
//---\|||/---------
//----\|/----------
//#####c###########
//----/|\----------
//---/|||\---------
//--/|||||\--------
//-/|||||||\-------


inline bool is_obstacle_in_direction(map_position corner, Direction dir, const Graph &graph)
{
  moving_direction(dir, corner, graph);
  return graph.is_obstacle(corner);
}

bool is_useful_nearby_corner(map_position pos, map_position corner, const Graph &graph)
{
  xyLoc pos_xy = graph.loc(pos);
  xyLoc corner_xy = graph.loc(corner);

  for (Direction dir : get_cardinal_directions())
  {
    if (is_obstacle_in_direction(corner, dir, graph))
    {
      Direction clockwise_diagonal = get_45_degrees_clockwise(dir);
      Direction clockwise_straight = get_45_degrees_clockwise(clockwise_diagonal);
      Direction anticlockwise_diagonal = get_45_degrees_anticlockwise(dir);
      Direction anticlockwise_straight = get_45_degrees_anticlockwise(anticlockwise_diagonal);

      if (!is_obstacle_in_direction(corner, clockwise_diagonal, graph))
      {
        if (in_direction_from_point(corner_xy, pos_xy, clockwise_diagonal))
          return true;

        if (within_45_degrees_clockwise_from_point(corner_xy, pos_xy, clockwise_diagonal))
          return true;

        if (in_direction_from_point(corner_xy, pos_xy, anticlockwise_straight))
          return true;

        if (within_45_degrees_anticlockwise_from_point(corner_xy, pos_xy, anticlockwise_straight))
          return true;
      }

      if (!is_obstacle_in_direction(corner, anticlockwise_diagonal, graph))
      {
        if (in_direction_from_point(corner_xy, pos_xy, anticlockwise_diagonal))
          return true;

        if (within_45_degrees_anticlockwise_from_point(corner_xy, pos_xy, anticlockwise_diagonal))
          return true;

        if (in_direction_from_point(corner_xy, pos_xy, clockwise_straight))
          return true;

        if (within_45_degrees_clockwise_from_point(corner_xy, pos_xy, clockwise_straight))
          return true;
      }
    }
  }

  return false;
}


void PreprocessingData::_replace_removed_corner(const map_position p, const map_position c, std::vector<corner_index> &nearby_corner_indices, int & num_added, int & num_added_more_than_removed)
{
  // When a corner is removed, a new corner may need to be added, because
  // that corner may not have been direct reachable due to the corner that
  // has just now been removed, but it may now effectively be direct
  // reachable
  int num_added_to_replace_this_index = 0;
  for (corner_index i : _point_to_nearby_corner_indices[c])
  {
    // Check if distance to new corner is optimal through old
    if (graph.octile_distance(p, _corners[i]) != graph.octile_distance(p, c) + graph.octile_distance(c, _corners[i]))
      continue;
    
    // Check if new corner is not already contained in nearby corner indices
    if (std::find(nearby_corner_indices.begin(), nearby_corner_indices.end(), i) != nearby_corner_indices.end())
      continue;
    
    // Check if new corner is safe-reachable
    if (!graph.safe_reachable(p, _corners[i]))
      continue;
    
    nearby_corner_indices.push_back(i);
    num_added_to_replace_this_index++;
    num_added++;
  }
  if (num_added_to_replace_this_index > 1)
  {
    num_added_more_than_removed++;
  }
}

void PreprocessingData::_remove_useless_nearby_corners()
{
  int num_removed = 0;
  int num_retained = 0;
  int num_added = 0;
  int num_added_more_than_removed = 0;
  
  map_position p;
  for (p = 0; p < graph.num_positions(); p++)
  {
    if (p % 10000 == 0)
      std::cout << p << (p != graph.num_positions() - 1 ? ", " : "") << std::flush;

    auto &nearby_corner_indices = _point_to_nearby_corner_indices[p];
    
    for (unsigned int which_nearby_corner = 0; which_nearby_corner < nearby_corner_indices.size();)
    {
      corner_index nearby_corner_index = nearby_corner_indices[which_nearby_corner];
      map_position c = _corners[nearby_corner_index];
      
      if (!is_useful_nearby_corner(p, c, graph))
      {
        nearby_corner_indices.erase(nearby_corner_indices.begin() + which_nearby_corner);
        num_removed++;
        
        _replace_removed_corner(p, c, nearby_corner_indices, num_added, num_added_more_than_removed);
      }
      else
      {
        which_nearby_corner++;
        num_retained++;
      }
    }
  }
  if ((p - 1) % 10000 != 0)
    std::cout << p - 1;
  std::cout << std::endl;
  
  std::cout << "Num useless nearby corners removed: " << num_removed << std::endl;
  std::cout << "Num useful nearby corners retained: " << num_retained << std::endl;
  std::cout << "Num nearby corners added: " << num_added << std::endl;
  
  if (num_added_more_than_removed > 0)
  {
    std::cout << "-----------------------------------------------------------------------" << std::endl;
    std::cout << "WARNING: more than one corner added to replace corner, for " << num_added_more_than_removed << " corners" << std::endl;
    std::cout << "Maybe we shouldn't have removed these corners." << std::endl;
    std::cout << "-----------------------------------------------------------------------" << std::endl;
  }
}

void PreprocessingData::preprocess()
{
  Timer t;
  double total_time = 0;

  // Compute corners
  std::cout << "Computing corners" << std::endl;
  t.StartTimer();
  _compute_corners();
  t.EndTimer();
  std::cout << "Corners computed in " << t.GetElapsedTime() << std::endl;
  total_time += t.GetElapsedTime();
  
  // Find nearby corners
  std::cout << "Finding nearby corners" << std::endl;
  t.StartTimer();
  _find_nearby_corners();
  t.EndTimer();
  std::cout << "Nearby corners found in " << t.GetElapsedTime() << std::endl;
  total_time += t.GetElapsedTime();
  
  // Find complete corner graph
  std::cout << "Finding complete corner graph" << std::endl;
  t.StartTimer();
  _find_complete_corner_graph(); 
  t.EndTimer(); 
  std::cout << "Complete corner graph found in " << t.GetElapsedTime() << std::endl;
  total_time += t.GetElapsedTime();

  // Push first corners as far as they will go
  std::cout << "Pushing first corners in corner graph" << std::endl;
  t.StartTimer();
  _push_corners_in_corner_graph(); 
  t.EndTimer(); 
  std::cout << "Corners pushed in " << t.GetElapsedTime() << std::endl;
  total_time += t.GetElapsedTime();

  // Remove pointless nearby corners
  std::cout << "Removing pointless nearby corners" << std::endl;
  t.StartTimer();
  _remove_useless_nearby_corners();
  t.EndTimer();
  std::cout << "Pointless nearby corners removed in " << t.GetElapsedTime() << std::endl;
  total_time += t.GetElapsedTime();
  
  std::cout << "Preprocessing complete" << std::endl;
  
  std::cout << "Total preprocessing time: " << total_time << std::endl;
  
  _output_debug_stats();
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
