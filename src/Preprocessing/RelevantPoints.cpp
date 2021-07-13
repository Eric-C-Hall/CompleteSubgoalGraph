#include "RelevantPoints.hpp"

#include "../Graph/XYLocStep.hpp"

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

/*void RelevantPoints::get_relevant_points(corner_index i, DivDirection divdirection, const Graph &graph)
{
  
}

void RelevantPoints::get_relevant_corners(corner_index i, DivDirection divdirection, const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners)
{
  for (corner_index j : nearby_corners.get_nearby_corner_indices(i))
  {
    xyLoc i_loc = graph.loc(corner_vector.get_corner(i));
    xyLoc j_loc = graph.loc(corner_vector.get_corner(j));

    if (is_point_relevant_assuming_safe_reachable(i_loc, divdirection, j_loc))
    {
      corner_to_divdirection_to_relevant_corners[i][divdirection].push_back(j);
    }
  }
}*/

void RelevantPoints::preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners)
{
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

  corner_to_outgoing_divdirection_to_relevant_points.resize(corner_vector.size());
  std::cout << "Computing outgoing relevant points: " << std::flush;
  for (i = 0; i < corner_vector.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << ", " << std::flush;
    corner_to_outgoing_divdirection_to_relevant_points[i].resize(num_divdirections());
    // TODO: Compute outgoing relevant points

  }
  std::cout << i - 1 << std::endl;

  // -------------------------------------------------------------------

  corner_to_outgoing_divdirection_to_relevant_corners.resize(corner_vector.size());
  std::cout << "Computing outgoing relevant corners: " << std::flush;
  for (i = 0; i < corner_vector.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << ", " << std::flush;
    corner_to_outgoing_divdirection_to_relevant_corners[i].resize(num_divdirections());
    // TODO: Compute outgoing relevant corners
  }
  std::cout << i - 1 << std::endl;
  
}

void RelevantPoints::print_outgoing_divdirections(const corner_index i, const DivDirection incoming_divdirection, Printer &printer, const Graph &graph, const CornerVector &corner_vector)
{
  const auto &divdirections = corner_to_incoming_divdirection_to_relevant_outgoing_divdirections[i][incoming_divdirection];
  
  const map_position corner = corner_vector.get_corner(i);
  printer.add_char('C', graph.loc(corner));
  for (DivDirection d : divdirections)
  {
    const map_position new_pos = graph.step_in_direction(corner, d);
    printer.add_highlight(Highlight(2), graph.loc(new_pos));
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

