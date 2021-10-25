#include "SmoothedGraph.hpp"
#include "../Graph/Directions.hpp"
#include "../Graph/CollarOperations.hpp"
#include "../PathCompetition/Timer.h"
#include "../Visualise/VisualisePreprocessed.hpp"

#include <random>
#include <algorithm>

void SmoothedGraph::preprocess(const Graph &graph, const CornerVector &corner_vector)
{
  is_blocked.clear();
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    is_blocked.push_back(graph.is_obstacle(p));
  }

  is_corner.clear();
  is_corner.resize(graph.num_positions(), false);
  for (corner_index i = 0; i < corner_vector.size(); i++)
    is_corner[corner_vector.get_corner(i)] = true;

  auto_smoothen_straight(graph, corner_vector);
  //auto_smoothen_diagonal();
}

void SmoothedGraph::remove_collar(const Graph &graph)
{
  CollarOperations::remove_collar_map(is_blocked, graph);
  CollarOperations::remove_collar_map(is_corner, graph);

  CollarOperations::remove_collar_each(last_added_obstacles, graph);
  CollarOperations::remove_collar_each(removable_corners, graph);
  CollarOperations::remove_collar_each(addable_corners, graph);
}

void SmoothedGraph::auto_smoothen_straight(const Graph &graph, const CornerVector &corner_vector)
{
  Timer t;
  t.StartTimer();

  std::cout << "Smoothening straight: ";
  corner_index i;
  for (i = 0; i < corner_vector.size(); i++)
  {
    map_position c = corner_vector.get_corner(i);
    if (i % 100 == 0)
      std::cout << i << ", " << std::flush;

    for (const Direction diagonal_direction : get_diagonal_directions())
    {
      for (const bool go_clockwise : {true, false})
      {
        // Step 1: Add obstacles
        smoothen_straight(c, diagonal_direction, go_clockwise, graph);

        // Step 2: Check effect of adding obstacles on corners
        find_newly_removable_corners(graph);

        // Step 3: Confirm/Revert
        if (smoothen_acceptable())
        {
          confirm_add_obstacles(graph);
        }
        else
        {
          undo_add_obstacles();
        }
      }
    }
  }
  std::cout << i-1 << std::endl;

  t.EndTimer();
  std::cout << "Smoothening straight took " << t.GetElapsedTime() << std::endl;
  std::cout << "Number of corners after smoothening: " << calculate_corners(graph).size() << std::endl;
  return;
}

void SmoothedGraph::smoothen_straight(const map_position c, const Direction diagonal_direction, const bool go_clockwise, const Graph &graph)
{
  const Direction side_direction_clock = get_45_degrees_clockwise(diagonal_direction);
  const Direction side_direction_anticlock = get_45_degrees_anticlockwise(diagonal_direction);

  const map_position wall_pos = graph.step_in_direction(c, diagonal_direction);
  const map_position clock_pos = graph.step_in_direction(c, side_direction_clock);
  const map_position anticlock_pos = graph.step_in_direction(c, side_direction_anticlock);

  if (!is_blocked[wall_pos] || is_blocked[clock_pos] || is_blocked[anticlock_pos])
    return;

  const map_position first_new_wall_pos = (go_clockwise ? clock_pos : anticlock_pos);
  const Direction construct_direction = (go_clockwise ? get_90_degrees_clockwise(side_direction_clock) : get_90_degrees_anticlockwise(side_direction_anticlock));

  const unsigned int num_walls_added = create_obstacles_in_direction(first_new_wall_pos, c, construct_direction, graph);
  const map_position behind_wall_pos = graph.step_in_direction(first_new_wall_pos, (go_clockwise ? side_direction_clock : side_direction_anticlock));
  const map_position before_behind_wall_pos = graph.step_in_direction(behind_wall_pos, get_opposite_direction(construct_direction));

  bool success;
  success = flood_fill_obstacles_in_direction(before_behind_wall_pos, construct_direction, num_walls_added + 2, graph);
  if (!success)
  {
    undo_add_obstacles();
    return;
  }
}

// Create obstacles in a certain direction until closing it off. It can close off in two ways.
//
// ------------------------------
// Closing off by hitting a wall:
// ------------------------------
//
// #######################################
// ######                              #
// ######------------------------------#
// #     X
//
// ------------------------------------------
// Closing off by passing just behind a wall:
// ------------------------------------------
//
// #######################################
// ########
// ########---------------
//         X             #########
//                      #############
//
// Returns the number of walls added

unsigned int SmoothedGraph::create_obstacles_in_direction(map_position construct_wall_pos, map_position before_wall_pos, Direction construct_direction, const Graph &graph)
{
  assert (!is_blocked[construct_wall_pos]);

  unsigned int num_walls_added = 0;
  while (true)
  {
    // Add obstacle
    add_obstacle(construct_wall_pos, graph);
    num_walls_added++;

    // Check if hit wall
    construct_wall_pos = graph.step_in_direction(construct_wall_pos, construct_direction);
    if (is_blocked[construct_wall_pos])
      break;

    // Check if passed just behind wall
    if (is_blocked[before_wall_pos])
      break;
    before_wall_pos = graph.step_in_direction(before_wall_pos, construct_direction);
  }
  return num_walls_added;
}

// Starts a flood fill from each location in a line of length dist in direction dir originating at the origin.
// So the number of locations we will start a flood fill from is equal to dist.
// Returns false if any of the flood fills fails
bool SmoothedGraph::flood_fill_obstacles_in_direction(const map_position origin, Direction dir, unsigned int dist, const Graph &graph)
{
  map_position curr = origin;
  for (unsigned int i = 0; i < dist; i++)
  {
    bool success = flood_fill_obstacles(curr, graph);
    if (!success)
    {
      return false;
    }
    curr = graph.step_in_direction(curr, dir);
  }
  return true;
}

bool SmoothedGraph::flood_fill_obstacles(map_position origin, const Graph &graph)
{
  std::vector<map_position> boundary;
  boundary.push_back(origin);
  while (!boundary.empty())
  {
    map_position p = boundary.back();
    boundary.pop_back();

    if (is_blocked[p])
    {
      continue;
    }

    add_obstacle(p, graph);

    for (Direction dir : get_directions())
    {
      boundary.push_back(graph.step_in_direction(p, dir));
    }

    if (boundary.size() > MAX_FLOOD_FILL_BOUNDARY_SIZE)
    {
      return false;
    }
  }
  return true;
}

void SmoothedGraph::add_obstacle(map_position p, const Graph &graph)
{
  assert (p < graph.num_positions());
  assert (!is_blocked[p]);
  last_added_obstacles.push_back(p);
  is_blocked[p] = true;
}

void SmoothedGraph::add_obstacles(const std::vector<map_position> &new_obstacles, const Graph &graph)
{
  for (map_position obstacle : new_obstacles)
  {
    add_obstacle(obstacle, graph);
  }
}

// TODO: This is basically copy/pasted from CornerVector.cpp, perhaps this can be done in a more nice way
bool SmoothedGraph::calculate_is_corner(map_position p, const Graph &graph)
{
  // Obstacles cannot be corners
  if (is_blocked[p])
  {
    return false;
  }

  // Check each diagonal direction to see if this is a corner for that direction
  for (Direction dir : get_diagonal_directions())
  {
    Direction clock_dir = get_45_degrees_clockwise(dir);
    Direction anticlock_dir = get_45_degrees_anticlockwise(dir);

    map_position diag_pos = graph.step_in_direction(p, dir);
    map_position clock_pos = graph.step_in_direction(p, clock_dir);
    map_position anticlock_pos = graph.step_in_direction(p, anticlock_dir);

    if (is_blocked[diag_pos] && !is_blocked[clock_pos] && !is_blocked[anticlock_pos])
    {
      return true;
    }
  }

  // If not a corner for any diagonal direction, then it is not a corner
  return false;
}

std::vector<map_position> SmoothedGraph::calculate_corners(const Graph &graph)
{
  std::vector<map_position> return_value;
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    const bool calculated_value = calculate_is_corner(p, graph);
    const bool stored_value = is_corner[p];
    assert (calculated_value == stored_value);
    if (stored_value)
    {
      assert (!is_blocked[p]);
      return_value.push_back(p);
    }
  }
  return return_value;
}

void SmoothedGraph::find_newly_removable_corners(const Graph &graph)
{
  for (const map_position p : last_added_obstacles)
  {
    assert(p < graph.num_positions());
    for (const Direction dir : get_directions())
    {
      map_position neighbour = graph.step_in_direction(p,dir);
      assert(neighbour < graph.num_positions());

      const bool current_is_corner = is_corner[neighbour];
      const bool new_is_corner = calculate_is_corner(neighbour, graph);

      if (current_is_corner && !new_is_corner)
      {
        removable_corners.insert(neighbour);
      }

      if (!current_is_corner && new_is_corner)
      {
        addable_corners.insert(neighbour);
      }
    }
  }
}

bool SmoothedGraph::smoothen_acceptable() const
{
  if (removable_corners.size() <= addable_corners.size())
  {
    return false;
  }

  unsigned int ratio = last_added_obstacles.size() / (removable_corners.size() - addable_corners.size());

  return ratio < 10;
}

void SmoothedGraph::undo_add_obstacles()
{
  remove_obstacles(last_added_obstacles);
  last_added_obstacles.clear();
  removable_corners.clear();
  addable_corners.clear();
}

void SmoothedGraph::confirm_add_obstacles(const Graph &graph)
{
  last_added_obstacles.clear();

  // Remove desired corners
  for (const map_position c : removable_corners)
  {
    assert (c < graph.num_positions());
    assert(is_corner[c]);
    is_corner[c] = false;
  }
  removable_corners.clear();

  // Add desired corners
  for (map_position c : addable_corners)
  {
    assert (c < graph.num_positions());
    assert(!is_corner[c]);
    is_corner[c] = true;
  }
  addable_corners.clear();
}

void SmoothedGraph::remove_obstacles(const std::vector<map_position> &removed_obstacles)
{
  for (map_position obstacle : removed_obstacles)
  {
    assert(is_blocked[obstacle]);
    is_blocked[obstacle] = false;
  }
}

void SmoothedGraph::print(Printer &printer, const Graph &graph) const
{
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    if (is_blocked[p] && !graph.is_obstacle(p))
      printer.add_char('Z', graph.loc(p));

    if (is_corner[p])
      printer.add_char('T', graph.loc(p));
  }
}

void SmoothedGraph::print(const Graph &graph) const
{
  Printer printer;
  graph.print(printer);
  print(printer, graph);
  printer.print();
}

/*void SmoothedGraph::auto_smoothen_diagonal()
{
  Timer t;
  t.StartTimer();

  auto corner_copy = _corners;
  int i = 0;
  std::cout << "Smoothening diagonal" << std::endl;
  for (map_position c : corner_copy)
  {
    if (i % 100 == 0)
      std::cout << (i == 0 ? "" : ", ") << i << std::flush;
    i++;

    for (bool b : {true, false})
    {
      // Step 1: Add obstacles
      smoothen_diagonal(c, b);

      // Step 2: Check effect of adding obstacles on corners
      find_newly_removable_corners();
      
      // Step 3: Confirm/Revert

      if (smoothen_acceptable())
      {
        confirm_add_obstacles();
      }
      else
      {
        undo_add_obstacles();
      }
    }
  }
  std::cout << std::endl;

  t.EndTimer();
  std::cout << "Smoothening diagonal took " << t.GetElapsedTime() << std::endl;
  return;
}

// -------------
// Smoothen diagonal
//
// # denotes obstacle
// - denotes free space
// X denotes corner
// O denotes new obstacle.
// ? denotes place to check if an obstacle or not
// F dentoes flood-fill location
//
// If either ? or O is an obstacle, then we may have closed off an area, so we should stop.
// Exception: the very first O can be an obstacle.
//
// -----FOO?
// ------FOO?
// -------FOOX
// --------FO#
// ---------F------

void SmoothedGraph::smoothen_diagonal(const map_position c, const bool use_clockwise_diagonal)
{
  // Find direction of wall
  Direction wall_direction = find_wall_direction(c);

  if (wall_direction == Dir_None)
    return;

  // Find step directions
  Direction dir1 = get_opposite_direction(wall_direction);
  Direction dir2 = (use_clockwise_diagonal ? get_n_steps_clockwise<2>(wall_direction) : get_n_steps_anticlockwise<2>(wall_direction));
  Direction diagonal = (use_clockwise_diagonal ? get_n_steps_clockwise<3>(wall_direction) : get_n_steps_anticlockwise<3>(wall_direction));
  
  // Find first obstacle and first floodfill position
  map_position first_add_obstacle = graph.step_in_direction(graph.step_in_direction(c, wall_direction), dir2);
  map_position first_floodfill_position = graph.step_in_direction(first_add_obstacle, wall_direction);

  // Create diagonal wall
  unsigned int steps = 2 + create_obstacles_along_diagonal(first_add_obstacle, dir1, dir2);
  
  // Flood fill behind wall
  bool success = flood_fill_obstacles_in_direction(first_floodfill_position, diagonal, steps);
  if (!success)
  {
    undo_add_obstacles();
    return;
  }
}

unsigned int SmoothedGraph::create_obstacles_along_diagonal(map_position first, Direction dir1, Direction dir2)
{
  unsigned int num_diagonal = 0;

  // First obstacle might already be obstacle, and we should not end immediately if it is
  if (!_obstacles[first])
    add_obstacle(first);

  map_position curr = first;
  while (true)
  {
    // Step once in dir1, add obstacle
    curr = graph.step_in_direction(first, dir1);
    if (_obstacles[curr])
      break;
    add_obstacle(curr);
    
    // Check if obstacle is further in dir1 
    map_position check_pos = graph.step_in_direction(curr, dir1);
    if (_obstacles[check_pos])
      break;
      
    // Step once in dir2, add obstacle.
    curr = graph.step_in_direction(curr, dir2);
    if (_obstacles[curr])
      break;
    add_obstacle(curr);
    
    // Keep track of number of diagonal steps taken
    num_diagonal++;    
  }
  
  return num_diagonal;
}*/
