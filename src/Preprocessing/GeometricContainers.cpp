#include "GeometricContainers.hpp"

#include "../Graph/Directions.hpp"
#include "../Graph/ExactDistance.hpp"
#include "../Graph/XYLocStep.hpp"

Bound GeometricContainersOutgoing::search_relevant(const corner_index i, const DivDirection outgoing_divdirection, const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points)
{
  Bound return_value = get_immediate_bound(i, outgoing_divdirection);

  std::deque<corner_index> open;
  std::deque<DivDirection> outgoing_divdirections;
  std::vector<std::vector<bool>> closed(corner_vector.size(), std::vector<bool>(num_divdirections(), false));

  open.push_back(i);
  outgoing_divdirections.push_back(outgoing_divdirection);
  
  while (!open.empty())
  {
    const corner_index curr_index = open.back();
    open.pop_back();

    const DivDirection curr_outgoing_divdirection = outgoing_divdirections.back();
    outgoing_divdirections.pop_back();

    const exact_distance curr_dist = complete_corner_graph.get_exact_distance_between_corner_indices(i, curr_index);

    if (closed[curr_index][curr_outgoing_divdirection])
      continue;
    closed[curr_index][curr_outgoing_divdirection] = true;

    return_value = return_value.combine(get_immediate_bound(curr_index, curr_outgoing_divdirection));

    for (corner_index next_index : relevant_points.get_relevant_corners(i, curr_outgoing_divdirection))
    {
      const exact_distance next_dist = curr_dist + complete_corner_graph.get_exact_distance_between_corner_indices(curr_index, next_index);
      assert(next_dist >= complete_corner_graph.get_exact_distance_between_corner_indices(i, next_index));
      if (next_dist != complete_corner_graph.get_exact_distance_between_corner_indices(i, next_index))
        continue;

      const DivDirection next_incoming_divdirection = curr_outgoing_divdirection;
      for (DivDirection next_outgoing_divdirection : relevant_points.get_relevant_outgoing_divdirections(next_index, next_incoming_divdirection))
      {
        open.push_back(next_index);
        outgoing_divdirections.push_back(next_outgoing_divdirection);
      }
    }    
  }

  return return_value;
}

void GeometricContainersOutgoing::preprocess_immediate(const Graph &graph, const CornerVector &corner_vector, const RelevantPoints &relevant_points)
{
  corner_to_outgoing_direction_to_immediate_bound.resize(corner_vector.size());
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    corner_to_outgoing_direction_to_immediate_bound[i].reserve(num_divdirections());
    for (const DivDirection outgoing_divdirection : get_divdirections())
    {
      const xyLoc corner_loc = graph.loc(corner_vector.get_corner(i));
      Bound curr_bound(corner_loc, corner_loc);
      bool is_first_point = true;
      for (map_position p : relevant_points.get_relevant_points(i, outgoing_divdirection))
      {
        const xyLoc p_loc = graph.loc(p);
        const Bound new_bound(p_loc, p_loc);
        if (is_first_point)
          curr_bound = new_bound, is_first_point = false;
        else
          curr_bound = curr_bound.combine(new_bound);
      }

      // TODO: assumes that the divdirections are looped through in order, so that these are put in the correct places
      corner_to_outgoing_direction_to_immediate_bound[i].push_back(curr_bound);
    }
  }
}

void GeometricContainersOutgoing::preprocess_overall(const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points)
{
  corner_to_outgoing_direction_to_bound.resize(corner_vector.size());
  corner_index i;
  for (i = 0; i < corner_vector.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << ", " << std::flush;
    corner_to_outgoing_direction_to_bound[i].reserve(num_divdirections());
    for (const DivDirection outgoing_divdirection : get_divdirections())
    {
      Bound curr_bound = search_relevant(i,outgoing_divdirection, graph, corner_vector, complete_corner_graph, relevant_points);
      // TODO: assumes that the divdirections are looped through in order, so that these are put in the correct places
      corner_to_outgoing_direction_to_bound[i].push_back(curr_bound);
    }
  }
  std::cout << i - 1<< std::endl;
}

void GeometricContainersOutgoing::preprocess(const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points)
{
  preprocess_immediate(graph, corner_vector, relevant_points);
  preprocess_overall(graph, corner_vector, complete_corner_graph, relevant_points);
}

void Bound::print(Printer &printer, const Highlight &highlight)
{
  for (int x = lower_bound.x; x <= upper_bound.x; x++)
    for (int y = lower_bound.y; y <= upper_bound.y; y++)
      printer.add_highlight(highlight, xyLoc(x,y));
}

void GeometricContainersOutgoing::print_bound(const corner_index i, const DivDirection dir, Printer &printer)
{
  corner_to_outgoing_direction_to_bound[i][dir].print(printer, Highlight(1));
}

void GeometricContainersOutgoing::print_immediate_bound(const corner_index i, const DivDirection dir, Printer &printer)
{
  corner_to_outgoing_direction_to_immediate_bound[i][dir].print(printer, Highlight(2));
}

void GeometricContainersOutgoing::print_all_bounds(const Graph &graph, const CornerVector &corner_vector)
{
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    for (DivDirection divdirection : get_divdirections())
    {
      Printer printer;
      graph.print(printer);
      print_bound(i, divdirection, printer);
      print_immediate_bound(i, divdirection, printer);
      
      const xyLoc corner_loc = graph.loc(corner_vector.get_corner(i));
      printer.add_char('X', corner_loc);
      const xyLoc divdirection_loc = step_in_direction(corner_loc, divdirection);
      printer.add_char('$', divdirection_loc);

      printer.print();
    }
  }
}

