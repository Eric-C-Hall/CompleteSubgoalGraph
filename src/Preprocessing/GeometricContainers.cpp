#include "GeometricContainers.hpp"

#include "../Graph/Directions.hpp"
#include "../Graph/ExactDistance.hpp"
#include "../Graph/XYLocStep.hpp"
#include "../Utility/SaveLoad.hpp"

Bounds GeometricContainersOutgoing::search_relevant(const corner_index i, const DivDirection outgoing_divdirection, const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points)
{
  Bounds return_value = get_immediate_bounds(i, outgoing_divdirection);

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

    return_value = return_value.combine(get_immediate_bounds(curr_index, curr_outgoing_divdirection));

    for (corner_index next_index : relevant_points.get_relevant_corners(curr_index, curr_outgoing_divdirection))
    {
      const exact_distance next_dist = curr_dist + complete_corner_graph.get_exact_distance_between_corner_indices(curr_index, next_index);
      assert(next_dist >= complete_corner_graph.get_exact_distance_between_corner_indices(i, next_index));
      if (next_dist != complete_corner_graph.get_exact_distance_between_corner_indices(i, next_index))
        continue;

      const DivDirection next_incoming_divdirection = curr_outgoing_divdirection;
      for (DivDirection next_outgoing_divdirection : relevant_points.get_relevant_divdirections(next_index, next_incoming_divdirection))
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
  corner_to_outgoing_direction_to_immediate_bounds.clear();
  corner_to_outgoing_direction_to_immediate_bounds.resize(corner_vector.size());
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    corner_to_outgoing_direction_to_immediate_bounds[i].reserve(num_divdirections());
    for (const DivDirection outgoing_divdirection : get_divdirections())
    {
      const xyLoc corner_loc = graph.loc(corner_vector.get_corner(i));
      Bounds curr_bounds(corner_loc, corner_loc);
      bool is_first_point = true;
      for (map_position p : relevant_points.get_relevant_points(i, outgoing_divdirection))
      {
        const xyLoc p_loc = graph.loc(p);
        const Bounds new_bounds(p_loc, p_loc);
        if (is_first_point)
          curr_bounds = new_bounds, is_first_point = false;
        else
          curr_bounds = curr_bounds.combine(new_bounds);
      }

      // TODO: assumes that the divdirections are looped through in order, so that these are put in the correct places
      corner_to_outgoing_direction_to_immediate_bounds[i].push_back(curr_bounds);
    }
  }
}

void GeometricContainersOutgoing::preprocess_overall(const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points)
{
  corner_to_outgoing_direction_to_bounds.clear();
  corner_to_outgoing_direction_to_bounds.resize(corner_vector.size());
  corner_index i;
  for (i = 0; i < corner_vector.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << ", " << std::flush;
    corner_to_outgoing_direction_to_bounds[i].reserve(num_divdirections());
    for (const DivDirection outgoing_divdirection : get_divdirections())
    {
      Bounds curr_bounds = search_relevant(i,outgoing_divdirection, graph, corner_vector, complete_corner_graph, relevant_points);
      // TODO: assumes that the divdirections are looped through in order, so that these are put in the correct places
      corner_to_outgoing_direction_to_bounds[i].push_back(curr_bounds);
    }
  }
  std::cout << i - 1<< std::endl;
}

void GeometricContainersOutgoing::preprocess(const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points)
{
  preprocess_immediate(graph, corner_vector, relevant_points);
  preprocess_overall(graph, corner_vector, complete_corner_graph, relevant_points);
}

void Bounds::print(Printer &printer, const Highlight &highlight) const
{
  for (int x = lower_bound.x; x <= upper_bound.x; x++)
    for (int y = lower_bound.y; y <= upper_bound.y; y++)
      printer.add_highlight(highlight, make_xyLoc(x,y));
}

void GeometricContainersOutgoing::print_bounds(const corner_index i, const DivDirection dir, Printer &printer) const
{
  corner_to_outgoing_direction_to_bounds[i][dir].print(printer, Highlight(1));
}

void GeometricContainersOutgoing::print_immediate_bounds(const corner_index i, const DivDirection dir, Printer &printer) const
{
  corner_to_outgoing_direction_to_immediate_bounds[i][dir].print(printer, Highlight(2));
}

void GeometricContainersOutgoing::print_all_bounds(const Graph &graph, const CornerVector &corner_vector) const
{
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    for (DivDirection divdirection : get_divdirections())
    {
      Printer printer;
      graph.print(printer);
      print_bounds(i, divdirection, printer);
      print_immediate_bounds(i, divdirection, printer);
      
      const xyLoc corner_loc = graph.loc(corner_vector.get_corner(i));
      printer.add_char('X', corner_loc);
      const xyLoc divdirection_loc = step_in_direction(corner_loc, divdirection);
      printer.add_char('$', divdirection_loc);

      printer.print();
    }
  }
}

void GeometricContainersIncoming::convert_from(const GeometricContainersOutgoing geometric_containers_outgoing, const Graph &graph, const CornerVector &corner_vector, const RelevantPoints &relevant_points)
{
  std::cout << "Converting outgoing geometric containers to incoming geometric containers: ";
  corner_to_incoming_direction_to_bounds.resize(corner_vector.size());
  corner_index i;
  for (i = 0; i < corner_vector.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << ", " << std::flush;
    corner_to_incoming_direction_to_bounds[i].reserve(num_divdirections());
    for (const DivDirection incoming_divdirection : get_divdirections())
    {
      const xyLoc ci_loc = graph.loc(corner_vector.get_corner(i));
      Bounds bounds(ci_loc, ci_loc);
      bool bounds_initialized = false;
      for (const DivDirection outgoing_divdirection : relevant_points.get_relevant_divdirections(i, incoming_divdirection))
      {
        const Bounds new_bounds = geometric_containers_outgoing.get_bounds(i, outgoing_divdirection);
        if (!bounds_initialized)
        {
          bounds = new_bounds;
          bounds_initialized = true;
        }
        else
          bounds = bounds.combine(new_bounds);
      }
      corner_to_incoming_direction_to_bounds[i].push_back(bounds);
    }
  }
  std::cout << i - 1 << std::endl;
}

void Bounds::remove_collar()
{
  upper_bound.x--;
  upper_bound.y--;
  lower_bound.x--;
  lower_bound.y--;
}

void GeometricContainersIncoming::remove_collar(const Graph &graph)
{
  for (auto & incoming_direction_to_bounds : corner_to_incoming_direction_to_bounds)\
  {
    for (Bounds & bounds : incoming_direction_to_bounds)
    {
      bounds.remove_collar();
    }
  }
}

void Bounds::save(std::ostream &stream) const
{
  SaveLoad::save_as_binary(stream, upper_bound.x);
  SaveLoad::save_as_binary(stream, upper_bound.y);
  SaveLoad::save_as_binary(stream, lower_bound.x);
  SaveLoad::save_as_binary(stream, lower_bound.y);
}

void GeometricContainersIncoming::save(std::ostream &stream) const
{
  int num_bounds_saved = 0;
  for (const auto &incoming_direction_to_bounds : corner_to_incoming_direction_to_bounds)
  {
    for (const auto &bounds : incoming_direction_to_bounds)
    {
      bounds.save(stream);
      num_bounds_saved++;
    }
  }
  std::cout << num_bounds_saved << " bounds saved" << std::endl;
}

void GeometricContainersIncoming::load(std::istream &stream, const CornerVector &corner_vector)
{
  corner_to_incoming_direction_to_bounds.resize(corner_vector.size());
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    corner_to_incoming_direction_to_bounds[i].reserve(num_divdirections());
    for (DivDirection dir : get_divdirections())
    {
      (void)dir;
      Bounds bounds(make_xyLoc(-1,-1),make_xyLoc(-1,-1));
      bounds.load(stream);
      corner_to_incoming_direction_to_bounds[i].push_back(bounds);
    }
  }
}

void GeometricContainersIncoming::print_bounds(const corner_index i, const DivDirection dir, Printer &printer) const
{
  assert (i < corner_to_incoming_direction_to_bounds.size());
  assert (dir < corner_to_incoming_direction_to_bounds[i].size());
  corner_to_incoming_direction_to_bounds[i][dir].print(printer, Highlight(3));
}

void Bounds::load(std::istream &stream)
{
  SaveLoad::load_as_binary(stream, upper_bound.x);
  SaveLoad::load_as_binary(stream, upper_bound.y);
  SaveLoad::load_as_binary(stream, lower_bound.x);
  SaveLoad::load_as_binary(stream, lower_bound.y);
}
