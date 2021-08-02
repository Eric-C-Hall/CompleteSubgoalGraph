#include "VisualisePreprocessed.hpp"

#include <vector>
#include <cmath>

#include "../Graph/MapPosition.hpp"
#include "../Run/GetPath/GetPath.hpp"
#include "../Graph/Islands.hpp"
#include "../Graph/XYLocStep.hpp"
#include "Printer.hpp"
#include "../Preprocessing/CornerVectorSplittable.hpp"
#include "../Preprocessing/DegreeTwoCorners.hpp"
#include "../Preprocessing/SmoothedGraph.hpp"

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
  const NearbyCornersWithRelevant &nearby_corners_with_relevant = preprocessing_data.get_nearby_corners_with_relevant();
  const NearbyCornersWithNext &nearby_corners_with_next = preprocessing_data.get_nearby_corners_with_next();
  const GeometricContainersIncoming &geometric_containers_incoming = preprocessing_data.get_geometric_containers_incoming();

  static SmoothedGraph smoothed_graph;
  static NearbyCorners nearby_corners;
  static DegreeTwoCorners degree_two_corners;
  static GeometricContainersOutgoing geometric_containers_outgoing;
  static RelevantPoints relevant_points;
  static CornerVectorSplittable corner_vector_splittable;
  static bool preprocessing_done = false;
  if (!preprocessing_done)
  {
    smoothed_graph.preprocess(graph, corner_vector);
    nearby_corners.preprocess(graph, corner_vector);
    degree_two_corners.preprocess(graph, corner_vector, nearby_corners, complete_corner_graph);
    relevant_points.preprocess(graph, corner_vector, nearby_corners);
    corner_vector_splittable.preprocess(graph, corner_vector, relevant_points);
    preprocessing_done = true;
  }

  // Geometric containers outgoing take an especially long time to compute, so only compute them if you need to
  static bool geometric_containers_outgoing_preprocessing_done = false;
  if (args.show_outgoing_bounds && !geometric_containers_outgoing_preprocessing_done)
  {
    geometric_containers_outgoing.preprocess(graph, corner_vector, complete_corner_graph, relevant_points);
    geometric_containers_outgoing_preprocessing_done = true;
  }

  // TODO: Maybe don't compute this every time, on the other hand maybe it's not important to be efficient
  const Islands islands(graph);

  Printer printer;

  // Print graph
  graph.print(printer);

  // Print corners
  if (args.show_corners)
    corner_vector.print(printer, graph);

  // Print splittable corners
  if (args.show_splittable_corners)
    corner_vector_splittable.print(printer, graph);

  // Print degree two corners
  if (args.show_degree_two_corners)
    degree_two_corners.print(printer, graph, corner_vector);

  // Print smoothed graph
  if (args.show_smoothed_graph)
    smoothed_graph.print(printer, graph);

  // Print nearby corners
  if (args.show_nearby && cursors.size() > 0)
    nearby_corners.print_nearby(cursors[0], printer, graph, corner_vector);

  // Print nearby corners with relevant
  if (args.show_nearby_with_relevant && cursors.size() > 0)
    nearby_corners_with_relevant.print_nearby_with_relevant(cursors[0], printer, graph, corner_vector);

  // Print nearby corners with next
  if (args.show_nearby_with_next && cursors.size() > 0)
    nearby_corners_with_next.print_nearby_with_next(cursors[0], printer, graph, corner_vector);

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

  // Print outgoing bounds
  if (args.show_outgoing_bounds)
    if (selected_corner != corner_vector.size())
    {
      geometric_containers_outgoing.print_bounds(selected_corner, args.divdirection, printer);
      geometric_containers_outgoing.print_immediate_bounds(selected_corner, args.divdirection, printer);
    }

  // Print incoming bounds
  if (args.show_incoming_bounds)
    if (selected_corner != corner_vector.size())
      geometric_containers_incoming.print_bounds(selected_corner, args.divdirection, printer);

  // Print relevant corners
  if (args.show_relevant_corners)
    if (selected_corner != corner_vector.size())
      relevant_points.print_relevant_corners(selected_corner, args.divdirection, printer, graph, corner_vector);

  // Print relevant points
  if (args.show_relevant_points)
    if (selected_corner != corner_vector.size())
      relevant_points.print_relevant_points(selected_corner, args.divdirection, printer, graph);

  // Print relevant points for incoming divdirection
  if (args.show_incoming_to_relevant_points)
    if (selected_corner != corner_vector.size())
      for (DivDirection outgoing_divdirection : relevant_points.get_relevant_divdirections(selected_corner, args.divdirection))
        relevant_points.print_relevant_points(selected_corner, outgoing_divdirection, printer, graph);

  // Print relevant outgoing divdirections
  if (args.show_relevant_divdirections)
    if (selected_corner != corner_vector.size())
      relevant_points.print_outgoing_divdirections(selected_corner, args.divdirection, printer, graph, corner_vector);

  // Print which_nearby_corner
  if (cursors.size() > 0)
  {
    const auto &cursor_nearby_corners = nearby_corners.get_nearby_corner_indices(cursors[0]);
    if (args.which_nearby_corner >= 0 && args.which_nearby_corner < (int)cursor_nearby_corners.size())
    {
      const corner_index nearby_corner_index = cursor_nearby_corners[args.which_nearby_corner];
      const xyLoc nearby_corner_loc = graph.loc(corner_vector.get_corner(nearby_corner_index));
      printer.add_highlight(Highlight(5), nearby_corner_loc);
    }
  }

  printer.print();

  if (cursors.size() > 0)
    std::cout << "Selected map_position: " << cursors[0] << std::endl;
  if (selected_corner != corner_vector.size())
    std::cout << "Selected corner: " << selected_corner << std::endl;
  if (args.show_divdirection)
    std::cout << "Selected divdirection: " << args.divdirection << std::endl;
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
  const CornerVector &corner_vector = preprocessing_data.get_corner_vector();
  const NearbyCornersWithRelevant &nearby_corners_with_relevant = preprocessing_data.get_nearby_corners_with_relevant();

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
    else if (input == "map_position")
    {
      int cursor, pos;
      std::cin >> cursor >> pos;
      set_cursor_to_pos(cursors, graph, cursor, pos);
    }
    else if (input == "corners")
    {
      args.show_corners = !args.show_corners;
    }
    else if (input == "splittable_corners")
    {
      args.show_splittable_corners = !args.show_splittable_corners;
    }
    else if (input == "degree_two_corners")
    {
      args.show_degree_two_corners = !args.show_degree_two_corners;
    }
    else if (input == "smoothed_graph")
    {
      args.show_smoothed_graph = !args.show_smoothed_graph;
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
    else if (input == "nearby_with_relevant")
    {
      args.show_nearby_with_relevant = !args.show_nearby_with_relevant;
    }
    else if (input == "nearby_with_next")
    {
      args.show_nearby_with_next = !args.show_nearby_with_next;
    }
    else if (input == "swap")
    {
      int n, m;
      std::cin >> n >> m;
      std::swap(cursors[n], cursors[m]);
    }
    else if (input == "outgoing_bounds")
    {
      args.show_outgoing_bounds = !args.show_outgoing_bounds;
    }
    else if (input == "incoming_bounds")
    {
      args.show_incoming_bounds = !args.show_incoming_bounds;
    }
    else if (input == "divdirection" || input == "dd")
    {
      int new_divdirection;
      std::cin >> new_divdirection;
      // Ensure divdirection is in the correct range
      new_divdirection = ((new_divdirection % num_divdirections()) + num_divdirections()) % num_divdirections();
      args.show_divdirection = (args.divdirection != new_divdirection || !args.show_divdirection);
      args.divdirection = (DivDirection)new_divdirection;
    }
    else if (input == "relevant_corners")
    {
      args.show_relevant_corners = !args.show_relevant_corners;
    }
    else if (input == "relevant_divdirections")
    {
      args.show_relevant_divdirections = !args.show_relevant_divdirections;
    }
    else if (input == "relevant_points")
    {
      args.show_relevant_points = !args.show_relevant_points;
    }
    else if (input == "incoming_to_relevant_points")
    {
      args.show_incoming_to_relevant_points = !args.show_incoming_to_relevant_points;
    }
    else if (input == "which_nearby_corner" || input == "wnc")
    {
      std::cin >> args.which_nearby_corner;
    }
    else if (input == "go_nearby_corner" || input == "gnc")
    {
      if (cursors.size() > 0)
      {
        const auto &cursor_nearby_corners_with_relevant = nearby_corners_with_relevant.get_nearby_corner_indices_with_relevant(cursors[0]);
        if (args.which_nearby_corner >= 0 && args.which_nearby_corner < (int)cursor_nearby_corners_with_relevant.size())
        {
          const corner_index nearby_corner_index = cursor_nearby_corners_with_relevant[args.which_nearby_corner];
          const map_position nearby_corner_pos = corner_vector.get_corner(nearby_corner_index);
          args.divdirection = get_divdirection_between_points(graph.loc(cursors[0]), graph.loc(nearby_corner_pos));
          args.show_divdirection = true;
          set_cursor_to_pos(cursors, graph, 0, nearby_corner_pos);
        }
      }
    }

    print_graph(preprocessing_data, cursors, path, args);

    if (input == "help")
    {
      std::cout << std::endl;
      std::cout << "cursor n x y: moves the nth cursor to (x,y)" << std::endl;
      std::cout << "cursort n x y: translate the nth cursor by (x,y)" << std::endl;
      std::cout << "corner n m: move the nth cursor to select the mth corner" << std::endl;
      std::cout << "corners: display corners" << std::endl;
      std::cout << "map_position n p: move the nth cursor to select the pth position" << std::endl;
      std::cout << "splittable_corners: display splittable corners" << std::endl;
      std::cout << "degree_two_corners: display degree two corners" << std::endl;
      std::cout << "smoothed_graph: display smoothed graph" << std::endl;
      std::cout << "swap n m: swap the nth and mth cursors" << std::endl;
      std::cout << "compute: compute the path between cursors 1 and 2" << std::endl;
      std::cout << "nearby: show corners near the zeroth cursor" << std::endl;
      std::cout << "nearby_with_relevant: show corners with relevant near the zeroth cursor" << std::endl;
      std::cout << "nearby_with_next: show corners with next near the zeroth cursor" << std::endl;
      std::cout << "outgoing_bounds: show bounds with corner under cursor 0 and outgoing divdirection selected with divdirection command" << std::endl;
      std::cout << "incoming_bounds: show bounds with corner under cursor 0 and incoming divdirection selected with divdirection command" << std::endl;
      std::cout << "divdirection i: select/highlight the ith divdirection, or unhighlight it if already selected. Alias: dd" << std::endl;
      std::cout << "relevant_corners: show corners relevant to corner under cursor 0 with outgoing divdirection selected with divdirection command" << std::endl;
      std::cout << "relevant_divdirections: show outgoing divdirections relevant to incoming divdirection selected with divdirection command" << std::endl;
      std::cout << "relevant_points: show positions relevant to outgoing divdirection selected with divdirection command" << std::endl;
      std::cout << "incoming_to_relevant_points: show position relevant to incoming divdirection selected with divdirection command" << std::endl;
      std::cout << "which_nearby_corner n: select the nth nearby corner with relevant. Alias: wnc" << std::endl;
      std::cout << "go_nearby_corner: go to the selected nearby corner with relevant, and update divdirection to direction travelled. Alias: gnc" << std::endl;
      std::cout << std::endl;
    }

    VisualiseRequestInput(input);
  }
}
