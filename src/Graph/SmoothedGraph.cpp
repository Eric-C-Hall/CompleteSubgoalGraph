#include "SmoothedGraph.hpp"
#include "Directions.hpp"
#include "../Utility/Timer.h"

#include <random>
#include <algorithm>

// --------------------------------------------------------------------------------------
// TODO: _add_if_corner and _compute_corners have been copy/pasted from PreprocessingData
// This should probably be done in a nicer, neater way

// TODO: logic can be compressed using relative directions instead of absolute directions
bool SmoothedGraph::_is_corner(map_position p)
{
  if (_obstacles[p])
  {
    return false;
  }

  if (_obstacles[graph.left(p)])
  {
    if (_obstacles[graph.down(p)] || _obstacles[graph.up(p)])
    {
      return false;
    }
    if (!_obstacles[graph.left(graph.down(p))] || !_obstacles[graph.left(graph.up(p))])
    {
      return true;
    }
  }

  if (_obstacles[graph.right(p)])
  {
    if (_obstacles[graph.down(p)] || _obstacles[graph.up(p)])
    {
      return false;
    }
    if (!_obstacles[graph.right(graph.down(p))] || !_obstacles[graph.right(graph.up(p))])
    {
      return true;
    }
  }

  if (_obstacles[graph.up(p)])
  {
    if (_obstacles[graph.left(p)] || _obstacles[graph.right(p)])
    {
      return false;
    }
    if (!_obstacles[graph.up(graph.left(p))] || !_obstacles[graph.up(graph.right(p))])
    {
      return true;
    }
  }

  if (_obstacles[graph.down(p)])
  {
    if (_obstacles[graph.left(p)] || _obstacles[graph.right(p)])
    {
      return false;
    }
    if (!_obstacles[graph.down(graph.left(p))] || !_obstacles[graph.down(graph.right(p))])
    {
      return true;
    }
  }

  return false;
}

void SmoothedGraph::_add_if_corner(map_position p)
{
  if (_is_corner(p))
    _corners.push_back(p);
}

void SmoothedGraph::_compute_corners()
{
  _pos_to_corner_index.clear();
  _pos_to_corner_index.resize(graph.num_positions());

  _corners.clear();
  for (unsigned int x=1 ; x<graph.get_width()-1 ; x++)
  {
    for (unsigned int y=1 ; y<graph.get_height()-1 ; y++)
    {
      map_position p = graph.pos(x,y);
      _add_if_corner(p);
      if (_corners.size() > 0 && _corners.back() == p)
      {
        _pos_to_corner_index[p] = _corners.size() - 1;
      }
      else
      {
        _pos_to_corner_index[p] = MAX_POSSIBLE_CORNER_INDEX;
      }
    }
  }
}

// --------------------------------------------------------------------------------------

unsigned int SmoothedGraph::compute_num_free_spaces() const
{
  unsigned int return_value = 0;
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    if (!_obstacles[p])
    {
      return_value++;
    }
  }
  return return_value;
}

void SmoothedGraph::print(const SmoothedGraphPrintArgs &args) const
{
  for (unsigned int y = 0; y < graph.get_height(); y++)
  {
    for (unsigned int x = 0; x < graph.get_width(); x++)
    {
      map_position p = graph.pos(x,y);
      if (_obstacles[p])
      {
        if (!graph.is_obstacle(p))
        {
          std::cout << "$";
        }
        else
        {
          std::cout << "@";
        }
      }
      else
      {
        if (_pos_to_corner_index[p] == args.selected_corner)
        {
          std::cout << "\e[7m";
        }

        if (_pos_to_corner_index[p] != MAX_POSSIBLE_CORNER_INDEX)
        {
          std::cout << "!";
        }
        else if (_pos_to_initial_corner_index[p] != MAX_POSSIBLE_CORNER_INDEX)
        {
          std::cout << "?";
        }
        else
        {
          std::cout << ".";
        }
      }
      // Remove formatting
      std::cout << "\e[0m";
    }
    std::cout << "\n";
  }
  unsigned int curr_num_free_spaces = compute_num_free_spaces();
  std::cout << "Num corners:             " << _corners.size() << "\n";
  std::cout << "Num free spaces:         " << curr_num_free_spaces << "\n";
  std::cout << "Initial num corners:     " << _initial_corners.size() << "\n";
  std::cout << "Initial num free spaces: " << initial_num_free_spaces << "\n";
  std::cout << "\n";
  std::cout << "Percent of initial corners:     " << 100 * (double)_corners.size() / (double)_initial_corners.size() << "\n";
  std::cout << "Percent of initial free spaces: " << 100 * (double)curr_num_free_spaces / (double)initial_num_free_spaces << "\n";
  std::cout << std::flush;

}

void SmoothedGraph::auto_smoothen_straight()
{
  Timer t;
  t.StartTimer();

  auto corner_copy = _corners;
  int i = 0;
  std::cout << "Smoothening straight" << std::endl;
  for (map_position c : corner_copy)
  {
    if (i % 100 == 0)
      std::cout << (i == 0 ? "" : ", ") << i << std::flush;
    i++;

    // Step 1: Add obstacles
    smoothen_straight(c);

    // Step 2: Check effect of adding obstacles on corners
    find_newly_removable_corners();
   
    // Step 3: Confirm/Revert

    if (smoothen_straight_acceptable())
    {
      confirm_add_obstacles();
    }
    else
    {
      undo_add_obstacles();
    }
  }
  std::cout << std::endl;

  t.EndTimer();
  std::cout << "Smoothening straight took " << t.GetElapsedTime() << std::endl;
  return;

  // -------------------
  // Old version:
  // -------------------

  std::cout << "Smoothening straight: ";
  bool smoothen_successful = true;
  while (smoothen_successful)
  {
    std::cout << "|" << std::flush;
    smoothen_successful = false;

    // Order corners randomly
    std::vector<corner_index> random_corner_order(_corners.size());
    std::iota(random_corner_order.begin(), random_corner_order.end(), 0);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::shuffle(random_corner_order.begin(), random_corner_order.end(), gen);

    // Try smoothing corners in this random order. If reasonable, accept and continue
    for (corner_index i : random_corner_order)
    {
      // Step 1: Add obstacles
      smoothen_straight(_corners[i]);

      // Step 2: Check effect of adding obstacles on corners
      find_newly_removable_corners();

      // Step 3: Confirm/Revert

      if (smoothen_straight_acceptable())
      {
        confirm_add_obstacles();
        smoothen_successful = true;
        break;
      }
      else
      {
        undo_add_obstacles();
      }
    }
  }
  std::cout << std::endl;
}

void SmoothedGraph::smoothen_straight(const map_position c)
{
  // Find direction of wall
  Direction wall_direction = Dir_NE;
  for (Direction dir : get_cardinal_directions())
  {
    if (_obstacles[graph.step_in_direction(c, dir)])
    {
      wall_direction = dir;
      break;
    }
  }
  assert(wall_direction != Dir_NE);

  // Find wall
  map_position wall = graph.step_in_direction(c, wall_direction);
  map_position behind_wall = graph.step_in_direction(wall, wall_direction);

  // Find directions relative to wall
  Direction clockwise_direction = get_n_steps_clockwise<2>(wall_direction);
  Direction anticlockwise_direction = get_n_steps_anticlockwise<2>(wall_direction);

  // Find sideways new obstacles
  std::vector<map_position> sideways_obstacles;
  unsigned int clockwise_steps = create_obstacles_in_direction(wall, c, clockwise_direction);
  unsigned int anticlockwise_steps = create_obstacles_in_direction(wall, c, anticlockwise_direction);
  // TODO: I think 1 should be added to clockwise and anticlockwise steps to deal with situations in which only a diagonal is blocked off, rather than a straight?

  // Flood fill in correct direction for all new sideways obstacles
  flood_fill_obstacles_in_direction(behind_wall, clockwise_direction, clockwise_steps);
  flood_fill_obstacles_in_direction(behind_wall, anticlockwise_direction, anticlockwise_steps);
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
// #    X
//
// ------------------------------------------
// Closing off by passing just behind a wall:
// ------------------------------------------
//
// #######################################
// ########
// ########---------------
//        X              #########
//                      #############
//

unsigned int SmoothedGraph::create_obstacles_in_direction(map_position wall, map_position corner, Direction dir)
{
  unsigned int num_steps = 0;
  while (true)
  {
    // Check if hit wall
    wall = graph.step_in_direction(wall, dir);
    if (_obstacles[wall])
      break;

    // Add obstacle
    add_obstacle(wall);
    num_steps++;

    // Check if passed just behind wall
    corner = graph.step_in_direction(corner, dir);
    if (_obstacles[corner])
      break;
  };
  return num_steps;
}

void SmoothedGraph::flood_fill_obstacles_in_direction(map_position origin, Direction dir, unsigned int dist)
{
  for (unsigned int i = 0; i < dist; i++)
  {
    origin = graph.step_in_direction(origin, dir);
    flood_fill_obstacles(origin);
  }
}

void SmoothedGraph::flood_fill_obstacles(map_position origin)
{
  std::vector<map_position> boundary;
  boundary.push_back(origin);
  while (!boundary.empty())
  {
    map_position p = boundary.back();
    boundary.pop_back();

    if (_obstacles[p])
    {
      continue;
    }

    add_obstacle(p);

    for (Direction dir : get_directions())
    {
      boundary.push_back(graph.step_in_direction(p, dir));
    }
  }
}

void SmoothedGraph::add_obstacle(map_position p)
{
  last_added_obstacles.push_back(p);
  _obstacles[p] = true;
}

void SmoothedGraph::add_obstacles(const std::vector<map_position> &new_obstacles)
{
  for (map_position obstacle : new_obstacles)
  {
    add_obstacle(obstacle);
  }
}

void SmoothedGraph::find_newly_removable_corners()
{
  for (map_position p : last_added_obstacles)
  {
    for (Direction dir : get_directions())
    {
      map_position neighbour = graph.step_in_direction(p,dir);
      if (_pos_to_corner_index[neighbour] != MAX_POSSIBLE_CORNER_INDEX)
      {
        if (!_is_corner(neighbour))
          removable_corners.insert(neighbour);
      }
    }
  }
}

bool SmoothedGraph::smoothen_straight_acceptable() const
{
  if (removable_corners.size() == 0)
  {
    return false;
  }

  unsigned int ratio = last_added_obstacles.size() / removable_corners.size();

  return ratio < 10;
}

void SmoothedGraph::undo_add_obstacles()
{
  remove_obstacles(last_added_obstacles);
  last_added_obstacles.clear();
  removable_corners.clear();
}

void SmoothedGraph::confirm_add_obstacles()
{
  last_added_obstacles.clear();
  // Remove desired corners
  for (map_position c : removable_corners)
  {
    corner_index curr_index = _pos_to_corner_index[c];
    // Erase the current corner
    _corners.erase(_corners.begin() + curr_index);
    _pos_to_corner_index[c] = MAX_POSSIBLE_CORNER_INDEX;
    // Fix up the corner_indices that are now incorrect
    for (auto corner_iter = _corners.begin() + curr_index; corner_iter != _corners.end(); corner_iter++)
    {
      _pos_to_corner_index[*corner_iter]--;
    }
  }
  removable_corners.clear();
}

void SmoothedGraph::remove_obstacles(const std::vector<map_position> &removed_obstacles)
{
  for (map_position obstacle : removed_obstacles)
  {
    assert(!graph.is_obstacle(obstacle));
    _obstacles[obstacle] = false;
  }
}

SmoothedGraph::SmoothedGraph(const Graph &input_graph) : graph(input_graph)
{
  // Initialise obstacles
  _obstacles.resize(graph.num_positions());
  for (map_position p = 0; p < graph.num_positions(); p++)
  {
    _obstacles[p] = graph.is_obstacle(p);
  }

  // Find initial free spaces
  initial_num_free_spaces = compute_num_free_spaces();

  // Find initial corners
  _compute_corners();
  _initial_corners = _corners;

  // Find initial pos to corner index
  _pos_to_initial_corner_index = _pos_to_corner_index;
}

void GUIRequestInput(std::string &input)
{
  std::cout << "Type help for help. Type quit to quit." << std::endl;
  std::cin >> input;
}

void SmoothedGraph::gui()
{
  SmoothedGraphPrintArgs args;
  print(args);

  std::string input;
  GUIRequestInput(input);
  while (input != "quit")
  {
    if (input == "corner")
    {
      std::cin >> args.selected_corner;
    }
    else if (input == "smoothenstraight")
    {
      smoothen_straight(args.selected_corner);
    }
    else if (input == "autosmoothenstraight")
    {
      auto_smoothen_straight();
    }
    else if (input == "undo")
    {
      undo_add_obstacles();
    }

    print(args);

    if (input == "help")
    {
      std::cout << std::endl;
      std::cout << "corner n: select nth corner" << std::endl;
      std::cout << "smoothenstraight: smoothen the selected corner" << std::endl;
      std::cout << "autosmoothenstraight: smoothen appropriate corners automatically" << std::endl;
      std::cout << "undo: undo smoothenstraight" << std::endl;
      std::cout << std::endl;
    }

    GUIRequestInput(input);
  }
}
