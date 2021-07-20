#include "RelevantPoints.hpp"

#include "../Graph/XYLocStep.hpp"
#include "../Graph/Reachable.hpp"

// Relevant: A corner is relevant to a pair of points if they are not safe reachable from each other, but c is safe-reachable from each point. 
//
// ###########---/-----
// ###########--/------
// ###########-/-------
// -----------c
// ----------/ 
// ---------/|
// --------/-|
// 
// If strictly to the left of the lower diagonal, then include all at top
//
// If in bottom triangle, then include top triangle
//
// In other words, we have the following incoming directions to outgoing directions
//
// E -> N, NNE, NE, ENE
// ENE -> N, NNE, NE, ENE
// NE -> N, NNE
// NNE -> N, NNE
//
// S -> W, WSW, SW, SSW
// SSW -> W, WSW, SW, SSW
// SW -> W, WSW
// WSW -> W, WSW
//
void RelevantPoints::compute_relevant_divdirections(const corner_index i, const DivDirection corner_divdirection, const Graph &graph, const CornerVector &corner_vector)
{
  const map_position corner = corner_vector.get_corner(i);
  const map_position corner_wall_loc = graph.step_in_direction(corner, corner_divdirection);
  if (!graph.is_obstacle(corner_wall_loc))
    return;
  const map_position next_to_corner_wall_loc_a = graph.step_in_direction(corner, get_45_degrees_clockwise(corner_divdirection));
  const map_position next_to_corner_wall_loc_b = graph.step_in_direction(corner, get_45_degrees_anticlockwise(corner_divdirection));
  if (graph.is_obstacle(next_to_corner_wall_loc_a) || graph.is_obstacle(next_to_corner_wall_loc_b))
    return;

  std::vector<std::set<DivDirection>> & divdirection_to_divdirections = corner_to_incoming_divdirection_to_relevant_outgoing_divdirections[i];

  // See diagram and comments above for why these specific choices of outgoing directions for incoming directions are appropriate
  update_outgoing_divdirections(divdirection_to_divdirections, corner_divdirection, DivDir_E, std::vector<DivDirection>{DivDir_N, DivDir_NNE, DivDir_NE, DivDir_ENE});
  update_outgoing_divdirections(divdirection_to_divdirections, corner_divdirection, DivDir_ENE, std::vector<DivDirection>{DivDir_N, DivDir_NNE, DivDir_NE, DivDir_ENE});
  update_outgoing_divdirections(divdirection_to_divdirections, corner_divdirection, DivDir_NE, std::vector<DivDirection>{DivDir_N, DivDir_NNE});
  update_outgoing_divdirections(divdirection_to_divdirections, corner_divdirection, DivDir_NNE, std::vector<DivDirection>{DivDir_N, DivDir_NNE});

  update_outgoing_divdirections(divdirection_to_divdirections, corner_divdirection, DivDir_S, std::vector<DivDirection>{DivDir_W, DivDir_WSW, DivDir_SW, DivDir_SSW});
  update_outgoing_divdirections(divdirection_to_divdirections, corner_divdirection, DivDir_SSW, std::vector<DivDirection>{DivDir_W, DivDir_WSW, DivDir_SW, DivDir_SSW});
  update_outgoing_divdirections(divdirection_to_divdirections, corner_divdirection, DivDir_SW, std::vector<DivDirection>{DivDir_W, DivDir_WSW});
  update_outgoing_divdirections(divdirection_to_divdirections, corner_divdirection, DivDir_WSW, std::vector<DivDirection>{DivDir_W, DivDir_WSW});
}

void RelevantPoints::update_outgoing_divdirections(std::vector<std::set<DivDirection>> & divdirection_to_divdirections, const DivDirection corner_divdirection, const DivDirection relative_divdirection, const std::vector<DivDirection> &added_relative_divdirections)
{
  // As you can see in the diagram above, N is 2 to the right of the corner divdirection, so we need to add 2 to the relative div_direction to get the true relative divdirection
  
  for (DivDirection added_divdirection : added_relative_divdirections)
  {
    divdirection_to_divdirections[corner_divdirection + relative_divdirection + ((DivDirection)2)].insert(corner_divdirection + added_divdirection + ((DivDirection)2));
  }
}

void RelevantPoints::compute_relevant_points(const corner_index i, const Graph &graph, const CornerVector &corner_vector)
{
  const map_position corner = corner_vector.get_corner(i);
  for (map_position p : Reachable::get_safe_reachable_in_all_directions(graph, corner))
  {
    if (p == corner)
      continue;
    DivDirection towards_p_divdirection = get_divdirection_between_points(graph.loc(corner), graph.loc(p));
    
    corner_to_outgoing_divdirection_to_relevant_points[i][towards_p_divdirection].push_back(p);
  }
}

void RelevantPoints::compute_relevant_corners(const corner_index i, const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners)
{
  for (corner_index j : nearby_corners.get_nearby_corner_indices(corner_vector.get_corner(i)))
  {
    xyLoc i_loc = graph.loc(corner_vector.get_corner(i));
    xyLoc j_loc = graph.loc(corner_vector.get_corner(j));

    DivDirection towards_j_divdirection = get_divdirection_between_points(i_loc, j_loc);

    corner_to_outgoing_divdirection_to_relevant_corners[i][towards_j_divdirection].push_back(j);
  }
}

void RelevantPoints::preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners)
{
  corner_to_incoming_divdirection_to_relevant_outgoing_divdirections.clear();
  corner_to_incoming_divdirection_to_relevant_outgoing_divdirections.resize(corner_vector.size());
  std::cout << "Computing incoming to outgoing divdirections: " << std::flush;
  corner_index i;
  for (i = 0; i < corner_vector.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << ", " << std::flush;

    corner_to_incoming_divdirection_to_relevant_outgoing_divdirections[i].resize(num_divdirections());
    for (DivDirection corner_divdirection : get_diagonal_divdirections())
    {
      compute_relevant_divdirections(i, corner_divdirection, graph, corner_vector);
    }
  }
  std::cout << i - 1 << std::endl;

  // -------------------------------------------------------------------

  corner_to_outgoing_divdirection_to_relevant_points.clear();
  corner_to_outgoing_divdirection_to_relevant_points.resize(corner_vector.size());
  std::cout << "Computing outgoing relevant points: " << std::flush;
  for (i = 0; i < corner_vector.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << ", " << std::flush;
    corner_to_outgoing_divdirection_to_relevant_points[i].resize(num_divdirections());
    compute_relevant_points(i, graph, corner_vector);
  }
  std::cout << i - 1 << std::endl;

  // -------------------------------------------------------------------

  corner_to_outgoing_divdirection_to_relevant_corners.clear();
  corner_to_outgoing_divdirection_to_relevant_corners.resize(corner_vector.size());
  std::cout << "Computing outgoing relevant corners: " << std::flush;
  for (i = 0; i < corner_vector.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << ", " << std::flush;
    corner_to_outgoing_divdirection_to_relevant_corners[i].resize(num_divdirections());
    compute_relevant_corners(i, graph, corner_vector, nearby_corners);
  }
  std::cout << i - 1 << std::endl;
  
}

void RelevantPoints::print_outgoing_divdirections(const corner_index i, const DivDirection incoming_divdirection, Printer &printer, const Graph &graph, const CornerVector &corner_vector)
{
  const auto &outgoing_divdirections = corner_to_incoming_divdirection_to_relevant_outgoing_divdirections[i][incoming_divdirection];
  
  const map_position corner = corner_vector.get_corner(i);
  printer.add_char('C', graph.loc(corner));
  for (DivDirection d : outgoing_divdirections)
  {
    const map_position new_pos = graph.step_in_direction(corner, d);
    printer.add_highlight(Highlight(2), graph.loc(new_pos));

    for (corner_index j : corner_to_outgoing_divdirection_to_relevant_corners[i][d])
    {
      printer.add_highlight(Highlight(4), graph.loc(corner_vector.get_corner(j)));
    }
  }
  const map_position incoming_pos = graph.step_in_direction(corner, get_opposite_direction(incoming_divdirection));
  printer.add_highlight(Highlight(3), graph.loc(incoming_pos));
}

void RelevantPoints::print_all_outgoing_divdirections(const Graph &graph, const CornerVector &corner_vector)
{
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    for (DivDirection d : get_divdirections())
    {
      Printer printer;
      graph.print(printer);
      print_outgoing_divdirections(i, d, printer, graph, corner_vector);
      printer.print();
    }
  }
}

void RelevantPoints::print_relevant_corners(const corner_index i, const DivDirection outgoing_divdirection, Printer &printer, const Graph &graph, const CornerVector &corner_vector) const
{
  assert(corner_to_outgoing_divdirection_to_relevant_corners.size() > i);
  assert(corner_to_outgoing_divdirection_to_relevant_corners[i].size() > outgoing_divdirection);
  for (corner_index j : corner_to_outgoing_divdirection_to_relevant_corners[i][outgoing_divdirection])
  {
    printer.add_char('R', graph.loc(corner_vector.get_corner(j)));
  }
}

