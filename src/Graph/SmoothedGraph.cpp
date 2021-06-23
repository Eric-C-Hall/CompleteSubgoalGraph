#include "SmoothedGraph.hpp"
#include "Directions.hpp"
#include "../Utility/Timer.h"
#include "../Visualise/VisualisePreprocessed.hpp"

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
  for (unsigned int x=0 ; x<graph.get_width() ; x++)
  {
    for (unsigned int y=0 ; y<graph.get_height() ; y++)
    {
      map_position p = graph.pos(x,y);
      if (x != 0 && y != 0 && x != graph.get_width() - 1 && y != graph.get_height() - 1)
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

void SmoothedGraph::print_map(const SmoothedGraphPrintArgs &args) const
{
  for (unsigned int y = 0; y < graph.get_height(); y++)
  {
    for (unsigned int x = 0; x < graph.get_width(); x++)
    {
      map_position p = graph.pos(x,y);
      
      // Highlights
      if (_pos_to_corner_index[p] == args.selected_corner)
      {
        std::cout << "\e[7m";
      }
      if (p == args.selected_position)
      {
        std::cout << "\e[42m";
      }
      
      // Print the character
      if (args.show_pos_to_corner_index)
      {
        std::cout << int_to_drawn_char(_pos_to_corner_index[p]);
      }
      else if (_obstacles[p])
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
    std::cout << "\n" << std::flush;
  }
}

void SmoothedGraph::print(const SmoothedGraphPrintArgs &args) const
{
  if (args.print_map)
  {
    print_map(args);
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

    if (smoothen_acceptable())
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
}

Direction SmoothedGraph::find_wall_direction(const map_position c) const
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
  return wall_direction;
}

void SmoothedGraph::smoothen_straight(const map_position c)
{
  Direction wall_direction = find_wall_direction(c);

  // Find wall
  map_position wall = graph.step_in_direction(c, wall_direction);
  map_position behind_wall = graph.step_in_direction(wall, wall_direction);

  // Find directions relative to wall
  Direction clockwise_direction = get_n_steps_clockwise<2>(wall_direction);
  Direction anticlockwise_direction = get_n_steps_anticlockwise<2>(wall_direction);

  // Find sideways new obstacles
  unsigned int clockwise_steps = 2 + create_obstacles_in_direction(wall, c, clockwise_direction);
  unsigned int anticlockwise_steps = 2 + create_obstacles_in_direction(wall, c, anticlockwise_direction);

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
  map_position prev = origin;
  map_position curr = prev;
  for (unsigned int i = 0; i < dist; i++)
  {
    assert(graph.adjacent(prev, curr));
    flood_fill_obstacles(curr);
    prev = curr;
    curr = graph.step_in_direction(curr, dir);
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
  assert(p < graph.num_positions());
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
      assert(p < graph.num_positions());
      assert(neighbour < graph.num_positions());
      if (_pos_to_corner_index[neighbour] != MAX_POSSIBLE_CORNER_INDEX)
      {
        if (!_is_corner(neighbour))
          removable_corners.insert(neighbour);
      }
      else
      {
        if (_is_corner(neighbour))
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

void SmoothedGraph::confirm_add_obstacles()
{
  last_added_obstacles.clear();
  // Remove desired corners
  for (map_position c : removable_corners)
  {
    assert (c < graph.num_positions());
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

  // Add desired corners
  for (map_position c : addable_corners)
  {
    assert (c < graph.num_positions());
    _pos_to_corner_index[c] = _corners.size();
    _corners.push_back(c);
  }
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
    else if (input == "autosmoothendiagonal")
    {
      auto_smoothen_diagonal();
    }
    else if (input == "smoothendiagonal")
    {
      bool b;
      std::cin >> b;
      smoothen_diagonal(_corners[args.selected_corner], b);
    }
    else if (input == "pos")
    {
      std::cin >> args.selected_position;
    }
    else if (input == "loc")
    {
      unsigned int x;
      unsigned int y;
      std::cin >> x >> y;
      args.selected_position = graph.pos(xyLoc(x,y));
    }
    else if (input == "postocornerindex")
    {
      args.show_pos_to_corner_index = !args.show_pos_to_corner_index;
    }
    else if (input == "printmap")
    {
      args.print_map = !args.print_map;
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
      std::cout << "smoothendiagonal b: smoothen the selected corner diagonal with bool b" << std::endl;
      std::cout << "autosmoothendiagonal: smoothen appropriate corners automatically" << std::endl;
      std::cout << "pos p: select the position p" << std::endl;
      std::cout << "loc x y: select the position corresponding to the location (x,y)" << std::endl;
      std::cout << "postocornerindex: print postocornerindices" << std::endl;
      std::cout << "printmap: toggle printing of the map" << std::endl;
      std::cout << "undo: undo smoothenstraight" << std::endl;
      std::cout << std::endl;
    }

    GUIRequestInput(input);
  }
}

void SmoothedGraph::auto_smoothen_diagonal()
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
  flood_fill_obstacles_in_direction(first_floodfill_position, diagonal, steps);
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
}
