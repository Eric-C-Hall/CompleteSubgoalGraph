#include "VisualisePreprocessed.hpp"

#include <vector>
#include <cmath>

#include "../Graph/MapPosition.hpp"
#include "../Run/GetPath/GetPath.hpp"

void VisualiseRequestInput(std::string &input)
{
  std::cout << "Type help for help. Type quit to quit." << std::endl;
  std::cin >> input;
}

// TODO: looping through all cursors and path positions on every call to this seems slow.
// If performance becomes an issue, maybe these could all be stored in a vector from map_position to path/cursor
// Then it would only take constant time to check if is cursor or on path
// It's not worth doing this unless performance is actually an issue, though
void print_point(int x, int y, const PreprocessingData &preprocessing_data, const Graph &graph, const std::vector<map_position> &cursors, const std::vector<xyLoc> &path, const bool show_nearby, const bool show_num_nearby)
{
  map_position pos = graph.pos(x,y);

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

  if (graph.is_obstacle(pos))
  {
    std::cout << "@";
  }
  else if (show_num_nearby)
  {
    const int num_nearby = preprocessing_data.get_nearby_corners(pos).size();
    char c;
    if (num_nearby <= 9)
    {
      c = '0' + num_nearby;
    }
    else if (num_nearby <= 9 + 26)
    {
      c = 'a' + num_nearby - 9;
    }
    else if (num_nearby <= 9 + 26 + 26)
    {
      c = 'A' + num_nearby - (9 + 26);
    }
    else
    {
      c = '>';
    }

    std::cout << c;
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


void print_graph(const PreprocessingData &preprocessing_data, const Graph &graph, const std::vector<map_position> &cursors, const std::vector<xyLoc> &path, const bool show_nearby, const bool show_num_nearby)
{
  print_topbar(graph.get_width(), graph.get_height() - 1);

  for (unsigned int y = 0; y < graph.get_height(); y++)
  {
    print_sidebar(y, graph.get_height() - 1);
    for (unsigned int x = 0; x < graph.get_width(); x++)
    {
      print_point(x,y,preprocessing_data,graph,cursors,path,show_nearby,show_num_nearby);
    }
    std::cout << "\n";
  }
  std::cout << std::flush;
}

void set_cursor_to_pos(std::vector<map_position> &cursors, unsigned int which_cursor, map_position pos)
{
  if (which_cursor < 0)
  {
    std::cerr << "Error: negative cursor selected" << std::endl;
    return;
  }

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

    set_cursor_to_pos(cursors, which_cursor, graph.pos(x,y));
  }
  else
  {
    unsigned int old_x = graph.x(cursors[which_cursor]);
    unsigned int old_y = graph.y(cursors[which_cursor]);
    set_cursor_to_pos(cursors, which_cursor, graph.pos((int)old_x + x, (int)old_y + y));
  }
}

void Visualise(const PreprocessingData &preprocessing_data)
{
  const Graph &graph = preprocessing_data.get_graph();

  std::string input = "";
  std::vector<map_position> cursors;
  std::vector<xyLoc> path;
  bool show_nearby = false;
  bool show_num_nearby = false;

  print_graph(preprocessing_data, graph, cursors, path, show_nearby, show_num_nearby);
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
      set_cursor_to_pos(cursors, n, preprocessing_data.get_corners()[m]);
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
    else if (input == "swap")
    {
      int n, m;
      std::cin >> n >> m;
      std::swap(cursors[n], cursors[m]);
    }

    print_graph(preprocessing_data, graph, cursors, path, show_nearby, show_num_nearby);

    if (input == "help")
    {
      std::cout << std::endl;
      std::cout << "cursor n x y: moves the nth cursor to (x,y)" << std::endl;
      std::cout << "cursort n x y: translate the nth cursor by (x,y)" << std::endl;
      std::cout << "swap n m: swap the nth and mth cursors" << std::endl;
      std::cout << "compute: compute the path between cursors 1 and 2" << std::endl;
      std::cout << "nearby: show corners near the zeroth cursor" << std::endl;
      std::cout << "corner n m: move the nth cursor to select the mth corner" << std::endl;
      std::cout << "numnearby: show number of nearby corners" << std::endl;
      std::cout << std::endl;
    }

    VisualiseRequestInput(input);
  }
}
