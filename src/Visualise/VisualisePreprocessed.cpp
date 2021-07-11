#include "VisualisePreprocessed.hpp"

#include <vector>
#include <cmath>

#include "../Graph/MapPosition.hpp"
#include "../Run/GetPath/GetPath.hpp"
#include "../Graph/Islands.hpp"
#include "Printer.hpp"

void VisualiseRequestInput(std::string &input)
{
  std::cout << "Type help for help. Type quit to quit." << std::endl;
  std::cin >> input;
}

char int_to_drawn_char(int i)
{
  if (i < -1)
  {
    return '<';
  }
  if (i == -1)
  {
    return '~';
  }
  else if (i <= 9)
  {
    return '0' + i;
  }
  else if (i <= 10 + 25)
  {
    return 'a' + i - 10;
  }
  else if (i <= 10 + 26 + 25)
  {
    return 'A' + i - (10 + 26);
  }
  else
  {
    return '>';
  }
}


// -------------------------
// TODO: This is very messy, it's just a copy of an existing function in Preprocess.cpp. Need to find a way to clean this up.
bool is_useful_nearby_corner_for_direction_copy(const map_position pos, const map_position corner, const Graph &graph, const Direction dir)
{
  const bool is_dir_cardinal = is_cardinal_direction(dir);

  const Direction opposite_dir = get_opposite_direction(dir);
  map_position opposite_pos = corner;
  moving_direction(opposite_dir, opposite_pos, graph);
  const bool is_opposite_obstacle = graph.is_obstacle(opposite_pos);

  if (is_opposite_obstacle)
    return false;

  const Direction clockwise_obstacle_dir = (is_dir_cardinal ? get_n_steps_clockwise<2>(opposite_dir) : get_45_degrees_clockwise(opposite_dir));
  map_position clockwise_obstacle_pos = corner;
  moving_direction(clockwise_obstacle_dir, clockwise_obstacle_pos, graph);
  const bool is_clockwise_obstacle = graph.is_obstacle(clockwise_obstacle_pos);

  const Direction anticlockwise_obstacle_dir = (is_dir_cardinal ? get_n_steps_anticlockwise<2>(opposite_dir) : get_45_degrees_anticlockwise(opposite_dir));
  map_position anticlockwise_obstacle_pos = corner;
  moving_direction(anticlockwise_obstacle_dir, anticlockwise_obstacle_pos, graph);
  const bool is_anticlockwise_obstacle = graph.is_obstacle(anticlockwise_obstacle_pos);

  const xyLoc pos_loc = graph.loc(pos);
  const xyLoc corner_loc = graph.loc(corner);

  if (is_clockwise_obstacle)
  {
    const Direction anticlockwise_45 = get_45_degrees_anticlockwise(dir);
    const Direction anticlockwise_90 = get_45_degrees_anticlockwise(anticlockwise_45);

    if (in_direction_from_point(corner_loc, pos_loc, anticlockwise_45))
      return true;

    if (within_45_degrees_anticlockwise_from_point(corner_loc, pos_loc, dir))
      return true;

    if (!is_dir_cardinal)
    {
      if (in_direction_from_point(corner_loc, pos_loc, anticlockwise_90))
        return true;

      if (within_45_degrees_anticlockwise_from_point(corner_loc, pos_loc, anticlockwise_45))
        return true;
    }
  }

  if (is_anticlockwise_obstacle)
  {
    const Direction clockwise_45 = get_45_degrees_clockwise(dir);
    const Direction clockwise_90 = get_45_degrees_clockwise(clockwise_45);

    if (in_direction_from_point(corner_loc, pos_loc, clockwise_45))
      return true;

    if (within_45_degrees_clockwise_from_point(corner_loc, pos_loc, dir))
      return true;

    if (!is_dir_cardinal)
    {
      if (in_direction_from_point(corner_loc, pos_loc, clockwise_90))
        return true;

      if (within_45_degrees_clockwise_from_point(corner_loc, pos_loc, clockwise_45))
        return true;
    }
  }

  return false;
}
// -------------

// TODO: looping through all cursors and path positions on every call to this seems slow.
// If performance becomes an issue, maybe these could all be stored in a vector from map_position to path/cursor
// Then it would only take constant time to check if is cursor or on path
// It's not worth doing this unless performance is actually an issue, though
void print_point(int x, int y, const PreprocessingData &preprocessing_data, const Graph &graph, const std::vector<map_position> &cursors, const std::vector<xyLoc> &path, const PrintGraphArguments &args, const Islands &islands, const std::pair<xyLoc, xyLoc> bounds)
{
  map_position pos = graph.pos(x,y);

  // Highlight bounds

  if (args.show_bounds && cursors.size() > 0)
  {
    if (graph.is_point_in_bounds(xyLoc(x,y), bounds))
    {
      std::cout << "\e[45m";
    }
  }

  if (0 <= args.selected_island && args.selected_island < islands.get_num_islands())
  {
    if (graph.is_point_in_bounds(xyLoc(x,y), islands.get_island_bounds(args.selected_island)))
    {
      std::cout << "\e[41m";
    }
  }

  // Highlight cursors
  for (map_position cursor : cursors)
  {
    if (pos == cursor)
    {
      std::cout << "\e[7m";
    }
  }

  // Highlight path
  for (xyLoc l : path)
  {
    if (pos == graph.pos(l))
    {
      std::cout << "\e[42m";
    }
  }

  // Highlight middirection
  if (args.show_bounds && cursors.size() > 0)
  {
    // Highlight middirection
    if (pos == move_in_middirection(args.middirection, cursors[0], graph))
      std::cout << "\e[47m";
  }


  // Instead of using if, else if, etc, we should use ifs and keep track of has_printed to ensure don't print twice
  if (graph.is_obstacle(pos))
  {
    std::cout << "@";
  }
  else if (args.show_islands)
  {
    std::cout << int_to_drawn_char(islands.get_island_index(pos));
  }
  else
  {
    std::cout << "-";
  }

  // Remove formatting
  std::cout << "\e[0m";
}


void print_graph(const PreprocessingData &preprocessing_data, const std::vector<map_position> &cursors, const std::vector<xyLoc> &path, const PrintGraphArguments &args)
{
  const Graph &graph = preprocessing_data.get_graph();
  const CornerVector &corner_vector = preprocessing_data.get_corner_vector();
  const NearbyCorners &nearby_corners = preprocessing_data.get_nearby_corners();
  const CompleteCornerGraph &complete_corner_graph = preprocessing_data.get_complete_corner_graph();

  // TODO: Maybe don't compute this every time, on the other hand maybe it's not important to be efficient
  const Islands islands(graph);

  Printer printer;
  graph.print(printer);
  if (args.show_nearby && cursors.size() > 0)
  {
    nearby_corners.print_nearby(cursors[0], printer, graph, corner_vector);
  }
  printer.print();
}

void set_cursor_to_pos(std::vector<map_position> &cursors, const Graph &graph, unsigned int which_cursor, map_position pos)
{
  if (which_cursor < 0)
  {
    std::cerr << "Error: negative cursor selected" << std::endl;
    return;
  }

  if (pos > graph.num_positions())
    return;

  if (which_cursor >= cursors.size())
  {
    cursors.resize(which_cursor + 1, 0);
  }

  cursors[which_cursor] = pos;
}

void move_cursor(std::vector<map_position> &cursors, const Graph &graph, const std::string &input)
{
  unsigned int which_cursor;
  std::cin >> which_cursor;

  int x,y;
  std::cin >> x >> y;
  
  if (input == "cursor")
  {
    if (x < 0 || y < 0)
    {
      std::cerr << "Error: cursor set to negative position" << std::endl;
      return;
    }

    set_cursor_to_pos(cursors, graph, which_cursor, graph.pos(x,y));
  }
  else
  {
    unsigned int old_x = graph.x(cursors[which_cursor]);
    unsigned int old_y = graph.y(cursors[which_cursor]);
    set_cursor_to_pos(cursors, graph, which_cursor, graph.pos((int)old_x + x, (int)old_y + y));
  }
}

void Visualise(const PreprocessingData &preprocessing_data)
{
  const Graph &graph = preprocessing_data.get_graph();

  std::string input = "";
  std::vector<map_position> cursors;
  std::vector<xyLoc> path;

  PrintGraphArguments args;
  args.show_nearby               = false;
  args.show_num_nearby           = false;
  args.show_nearby_with_next     = false;
  args.show_num_nearby_with_next = false;
  args.show_islands              = false;
  args.show_bounds               = false;
  args.middirection              = Dir_NNE;
  args.which_nearby_corner       = 100000;

  print_graph(preprocessing_data, cursors, path, args);
  VisualiseRequestInput(input);

  while (input != "quit")
  {
    if (input == "cursor" || input == "cursort")
    {
      move_cursor(cursors, graph, input);
    }
    else if (input == "corner")
    {
      int n, m;
      std::cin >> n >> m;
      set_cursor_to_pos(cursors, graph, n, preprocessing_data.get_corner_vector().get_corner(m));
    }
    else if (input == "compute" && cursors.size() >= 2)
    {
      path.clear();
      GetPath(preprocessing_data, graph.loc(cursors[0]), graph.loc(cursors[1]), path);
    }
    else if (input == "nearby")
    {
      args.show_nearby = !args.show_nearby;
    }
    else if (input == "numnearby")
    {
      args.show_num_nearby = !args.show_num_nearby;
    }
    else if (input == "numnearbywithnext")
    {
      args.show_num_nearby_with_next = !args.show_num_nearby_with_next;
    }
    else if (input == "nearbywithnext")
    {
      args.show_nearby_with_next = !args.show_nearby_with_next;
    }
    else if (input == "islands")
    {
      args.show_islands = !args.show_islands;
    }
    else if (input == "island")
    {
      std::cin >> args.selected_island;
    }
    else if (input == "swap")
    {
      int n, m;
      std::cin >> n >> m;
      std::swap(cursors[n], cursors[m]);
    }
    else if (input == "bounds")
    {
      MidDirection old_middirection = args.middirection;
      unsigned int temp;
      std::cin >> temp;
      args.middirection = (MidDirection)temp;
      if (!args.show_bounds || old_middirection != args.middirection)
      {
        args.show_bounds = true;
      }
      else
      {
        args.show_bounds = false;
      }
    }
    else if (input == "whichnearbycorner")
    {
      std::cin >> args.which_nearby_corner;
    }

    print_graph(preprocessing_data, cursors, path, args);

    if (input == "help")
    {
      std::cout << std::endl;
      std::cout << "cursor n x y: moves the nth cursor to (x,y)" << std::endl;
      std::cout << "cursort n x y: translate the nth cursor by (x,y)" << std::endl;
      std::cout << "swap n m: swap the nth and mth cursors" << std::endl;
      std::cout << "compute: compute the path between cursors 1 and 2" << std::endl;
      std::cout << "nearby: show corners near the zeroth cursor" << std::endl;
      std::cout << "nearbywithnext: show corners near the zeroth cursor with a next corner" << std::endl;
      std::cout << "corner n m: move the nth cursor to select the mth corner" << std::endl;
      std::cout << "numnearby: show number of nearby corners" << std::endl;
      std::cout << "numnearbywithnext: show number of nearby corners with next" << std::endl;
      std::cout << "island i: select the ith island" << std::endl;
      std::cout << "islands: show islands" << std::endl;
      std::cout << "bounds i: show bounds. Uses i to determine which middirection to use" << std::endl;
      std::cout << "gonearbycorner: sets cursor 0 to the selected nearby corner" << std::endl;
      std::cout << "whichnearbycorner i: highlight the ith nearby corner" << std::endl;
      std::cout << std::endl;
    }

    VisualiseRequestInput(input);
  }
}
