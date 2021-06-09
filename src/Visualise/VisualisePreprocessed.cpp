#include "VisualisePreprocessed.hpp"

#include <vector>
#include <cmath>

#include "../Graph/MapPosition.hpp"
#include "../Run/GetPath/GetPath.hpp"
#include "../Graph/Islands.hpp"

void VisualiseRequestInput(std::string &input)
{
  std::cout << "Type help for help. Type quit to quit." << std::endl;
  std::cin >> input;
}

char int_to_drawn_char(int i)
{
  if (i <= 9)
  {
    return '0' + i;
  }
  else if (i <= 9 + 26)
  {
    return 'a' + i - 9;
  }
  else if (i <= 9 + 26 + 26)
  {
    return 'A' + i - (9 + 26);
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
void print_point(int x, int y, const PreprocessingData &preprocessing_data, const Graph &graph, const std::vector<map_position> &cursors, const std::vector<xyLoc> &path, const bool show_nearby, const bool show_num_nearby, const bool show_nearby_with_next, const bool show_islands, const Islands &islands, const bool show_bounds, const MidDirection middirection, const unsigned int which_nearby_corner)
{
  map_position pos = graph.pos(x,y);

  // Highlight bounds

  if (show_bounds && cursors.size() > 0)
  {
    for (corner_index i = 0; i < preprocessing_data.get_corners().size(); i++)
    {
      if (preprocessing_data.get_corners()[i] == cursors[0])
      {
        const std::pair<xyLoc, xyLoc> bounds = preprocessing_data.get_bounds(i,middirection);
        if (x >= bounds.first.x && y >= bounds.first.y && x <= bounds.second.x && y <= bounds.second.y)
        {
          std::cout << "\e[45m";
        }
      }
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

  // Highlight nearby
  if (show_nearby && cursors.size() > 0)
  {
    for (map_position c : preprocessing_data.get_nearby_corners(cursors[0]))
    {
      if (pos == c)
      {
        std::cout << "\e[43m";
      }
    }
  }

  // Highlight nearby with next
  if (show_nearby_with_next && cursors.size() > 0)
  {
    for (map_position c : preprocessing_data.get_nearby_corners_with_next(cursors[0]))
    {
      if (pos == c)
      {
        std::cout << "\e[44m";
      }
    }
  }

  // Highlight chosen nearby corner
  if (cursors.size() > 0)
  {
    for (unsigned int i = 0; i < preprocessing_data.get_num_nearby_corners_with_next(cursors[0]); i++)
    {
      if (pos == preprocessing_data.get_ith_nearby_corner_with_next(cursors[0], i))
      {
        if (i == which_nearby_corner)
          std::cout << "\e[46m";
      }
    }
  }

  // Highlight middirection
  if (show_bounds && cursors.size() > 0)
  {
    // Highlight middirection
    if (pos == move_in_middirection(middirection, cursors[0], graph))
      std::cout << "\e[47m";
  }


  // Instead of using if, else if, etc, we should use ifs and keep track of has_printed to ensure don't print twice
  if (graph.is_obstacle(pos))
  {
    std::cout << "@";
  }
  else if (show_num_nearby)
  {
    const int num_nearby = preprocessing_data.get_nearby_corners(pos).size();
    std::cout << int_to_drawn_char(num_nearby);
  }
  else if (show_islands)
  {
    std::cout << int_to_drawn_char(islands.get_island_index(pos));
  }
  else if (show_bounds && cursors.size() > 0)
  {
    bool has_printed = false;
    for (unsigned int i = 0; i < preprocessing_data.get_num_nearby_corners_with_next(cursors[0]); i++)
    {
      if (pos == preprocessing_data.get_ith_nearby_corner_with_next(cursors[0], i))
      {
        // Print the direction in which the corner under the cursor is relevant for the corner at pos
        for (Direction dir : get_directions())
        {
          if (is_useful_nearby_corner_for_direction_copy(pos, cursors[0], graph, dir))
          {
            std::cout << direction_to_char(dir);
            assert(!has_printed);
            has_printed = true;
            break;
          }
        }
        if (!has_printed)
        {
          std::cout << "+";
          has_printed = true;
        }
        break;
      }
    }
    if (!has_printed)
    {
      std::cout << "-";
      has_printed = true;
    }
  }
  else
  {
    std::cout << "-";
  }

  // Remove formatting
  std::cout << "\e[0m";
}

// The negative sign on negative numbers counts as a digit
int num_digits(int n)
{
  if (n == 0)
    return 1;

  if (n < 0)
    return num_digits(-n) + 1;

  int order = 0;

  while (std::pow(10, order) <= n)
  {
    order++;
  }

  return order;
}

void print_n_spaces(int n)
{
  while (n > 0)
  {
    std::cout << " ";
    n--;
  }
}

void print_sidebar(int y, unsigned int max_y)
{
  std::cout << y;

  int margin = num_digits(max_y) - num_digits(y) + 1;

  print_n_spaces(margin);
}

void print_topbar(const unsigned int width, const unsigned int max_y)
{
  print_n_spaces(num_digits(max_y) + 1);

  int i = 0;
  while ((int)width - i >= (int)num_digits(i))
  {
    std::cout << i;
    i += num_digits(i);
    std::cout << " ";
    i += 1;
  }
  std::cout << "\n";
}


void print_graph(const PreprocessingData &preprocessing_data, const Graph &graph, const std::vector<map_position> &cursors, const std::vector<xyLoc> &path, const bool show_nearby, const bool show_num_nearby, const bool show_nearby_with_next, const bool show_islands, const bool show_bounds, const MidDirection middirection, const unsigned int which_nearby_corner)
{
  // TODO: Maybe don't compute this every time, on the other hand maybe it's not important to be efficient
  const Islands islands(graph);

  // Print the graph
  print_topbar(graph.get_width(), graph.get_height() - 1);

  for (unsigned int y = 0; y < graph.get_height(); y++)
  {
    print_sidebar(y, graph.get_height() - 1);
    for (unsigned int x = 0; x < graph.get_width(); x++)
    {
      print_point(x,y,preprocessing_data,graph,cursors,path,show_nearby,show_num_nearby,show_nearby_with_next, show_islands, islands, show_bounds, middirection, which_nearby_corner);
    }
    std::cout << "\n";
  }
  std::cout << std::flush;
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
  // TODO: These sorts of things are common: we should abstract them into a class that does the work for us, so that whenever we add another of these we can add them quickly
  // We should make a class that encapsulates all input to the print_graph function, and use that to input to the print_graph function. Much easier to extend
  bool show_nearby = false;
  bool show_num_nearby = false;
  bool show_nearby_with_next = false;
  bool show_islands = false;
  bool show_bounds = false;

  unsigned int which_nearby_corner = 100000;
  MidDirection middirection = Dir_NNE;

  print_graph(preprocessing_data, graph, cursors, path, show_nearby, show_num_nearby, show_nearby_with_next, show_islands, show_bounds, middirection, which_nearby_corner);
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
      set_cursor_to_pos(cursors, graph, n, preprocessing_data.get_corners()[m]);
    }
    else if (input == "compute" && cursors.size() >= 2)
    {
      path.clear();
      GetPath(preprocessing_data, graph.loc(cursors[0]), graph.loc(cursors[1]), path);
    }
    else if (input == "nearby")
    {
      show_nearby = !show_nearby;
    }
    else if (input == "numnearby")
    {
      show_num_nearby = !show_num_nearby;
    }
    else if (input == "nearbywithnext")
    {
      show_nearby_with_next = !show_nearby_with_next;
    }
    else if (input == "islands")
    {
      show_islands = !show_islands;
    }
    else if (input == "swap")
    {
      int n, m;
      std::cin >> n >> m;
      std::swap(cursors[n], cursors[m]);
    }
    else if (input == "bounds")
    {
      MidDirection old_middirection = middirection;
      unsigned int temp;
      std::cin >> temp;
      middirection = (MidDirection)temp;
      if (!show_bounds || old_middirection != middirection)
      {
        show_bounds = true;
      }
      else
      {
        show_bounds = false;
      }
    }
    else if (input == "whichnearbycorner")
    {
      std::cin >> which_nearby_corner;
    }
    else if (input == "gonearbycorner" && cursors.size() > 0)
    {
      map_position go_pos = preprocessing_data.get_ith_nearby_corner_with_next(cursors[0], which_nearby_corner);
      middirection = get_middirection_between_points(graph.loc(cursors[0]), graph.loc(go_pos));
      set_cursor_to_pos(cursors, graph, 0, go_pos);
    }


    print_graph(preprocessing_data, graph, cursors, path, show_nearby, show_num_nearby, show_nearby_with_next, show_islands, show_bounds, middirection, which_nearby_corner);

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
      std::cout << "islands: show islands" << std::endl;
      std::cout << "bounds i: show bounds. Uses i to determine which middirection to use" << std::endl;
      std::cout << "gonearbycorner: sets cursor 0 to the selected nearby corner" << std::endl;
      std::cout << "whichnearbycorner i: highlight the ith nearby corner" << std::endl;
      std::cout << std::endl;
    }

    VisualiseRequestInput(input);
  }
}
