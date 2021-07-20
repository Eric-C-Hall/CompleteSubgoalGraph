#include "VisualisePreprocessed.hpp"

#include <vector>
#include <cmath>

#include "../Graph/MapPosition.hpp"
#include "../Run/GetPath/GetPath.hpp"
#include "../Graph/Islands.hpp"
#include "../Graph/XYLocStep.hpp"
#include "Printer.hpp"

void VisualiseRequestInput(std::string &input)
{
  std::cout << "Type help for help. Type quit to quit." << std::endl;
  std::cin >> input;
}

void print_graph(const PreprocessingData &preprocessing_data, const std::vector<map_position> &cursors, const std::vector<xyLoc> &path, const PrintGraphArguments &args)
{
  const Graph &graph = preprocessing_data.get_graph();
  const CornerVector &corner_vector = preprocessing_data.get_corner_vector();
  const CompleteCornerGraph &complete_corner_graph = preprocessing_data.get_complete_corner_graph();
  const NearbyCorners &nearby_corners = preprocessing_data.get_nearby_corners();

  static GeometricContainersOutgoing geometric_containers_outgoing;
  static RelevantPoints relevant_points;
  static bool preprocessing_done = false;
  if ((args.show_bounds || args.show_relevant_corners)
      && !preprocessing_done)
  {
    relevant_points.preprocess(graph, corner_vector, nearby_corners);
    geometric_containers_outgoing.preprocess(graph, corner_vector, complete_corner_graph, relevant_points);
    preprocessing_done = true;
  }

  // TODO: Maybe don't compute this every time, on the other hand maybe it's not important to be efficient
  const Islands islands(graph);

  Printer printer;

  // Print graph
  graph.print(printer);

  // Print nearby corners
  if (args.show_nearby && cursors.size() > 0)
    nearby_corners.print_nearby(cursors[0], printer, graph, corner_vector);

  // Print path
  for (xyLoc l : path)
    printer.add_highlight(Highlight(2), l);
  if (path.size() > 0)
  {
    printer.add_highlight(Highlight(4), path.front());
    printer.add_highlight(Highlight(3), path.back());
  }

  // Print cursors
  for (map_position p : cursors)
    if (p < graph.num_positions())
      printer.add_highlight(Highlight(1), graph.loc(p));

  // Print divdirection
  if (args.show_divdirection && cursors.size() > 0)
    printer.add_char('$', step_in_direction(graph.loc(cursors[0]), args.divdirection));

  // Find the index of the selected corner
  corner_index selected_corner = corner_vector.size();
  if (cursors.size() > 0)
    for (selected_corner = 0; selected_corner < corner_vector.size() && corner_vector.get_corner(selected_corner) != cursors[0]; selected_corner++)
      (void)0;

  // Print bounds
  if (args.show_bounds)
    if (selected_corner != corner_vector.size())
    {
      geometric_containers_outgoing.print_bound(selected_corner, args.divdirection, printer);
      geometric_containers_outgoing.print_immediate_bound(selected_corner, args.divdirection, printer);
    }
    

  // Print relevant corners
  if (args.show_relevant_corners)
    if (selected_corner != corner_vector.size())
      relevant_points.print_relevant_corners(selected_corner, args.divdirection, printer, graph, corner_vector);

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
    else if (input == "swap")
    {
      int n, m;
      std::cin >> n >> m;
      std::swap(cursors[n], cursors[m]);
    }
    else if (input == "bounds")
    {
      args.show_bounds = !args.show_bounds;
    }
    else if (input == "divdirection" || input == "dd")
    {
      int new_divdirection;
      std::cin >> new_divdirection;
      args.show_divdirection = (args.divdirection != new_divdirection || !args.show_divdirection);
      args.divdirection = (DivDirection)new_divdirection;
    }
    else if (input == "relevant_corners")
    {
      args.show_relevant_corners = !args.show_relevant_corners;
    }

    print_graph(preprocessing_data, cursors, path, args);

    if (input == "help")
    {
      std::cout << std::endl;
      std::cout << "cursor n x y: moves the nth cursor to (x,y)" << std::endl;
      std::cout << "cursort n x y: translate the nth cursor by (x,y)" << std::endl;
      std::cout << "corner n m: move the nth cursor to select the mth corner" << std::endl;
      std::cout << "swap n m: swap the nth and mth cursors" << std::endl;
      std::cout << "compute: compute the path between cursors 1 and 2" << std::endl;
      std::cout << "nearby: show corners near the zeroth cursor" << std::endl;
      std::cout << "bounds: show bounds with corner under cursor 0 and outgoing divdirection selected with divdirection command" << std::endl;
      std::cout << "divdirection i: select/highlight the ith divdirection, or unhighlight it if already selected. Alias: dd" << std::endl;
      std::cout << "relevant_corners: show corners relevant to corner under cursor 0 with outgoing divdirection selected with divdirection command" << std::endl;
      std::cout << std::endl;
    }

    VisualiseRequestInput(input);
  }
}
