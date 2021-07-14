#include "GeometricContainers.hpp"

#include "../Graph/Directions.hpp"
#include "../Graph/ExactDistance.hpp"

Bound GeometricContainersOutgoing::search_relevant(const corner_index i, const DivDirection outgoing_divdirection, const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points)
{
  Bound return_value = get_immediate_bound(i, outgoing_divdirection);

  std::deque<corner_index> open;
  std::deque<DivDirection> incoming_divdirection;
  std::vector<exact_distance> dist(corner_vector.size(), MAX_EXACT_DISTANCE);

  open.push_back(i);
  incoming_divdirection
  dist[i] = ZERO_EXACT_DISTANCE;
  
  while (!open.empty())
  {
    const corner_index curr_index = open.back();
    open.pop_back();

    const DivDirection incoming_divdirection = 

    if (dist[curr_index] == complete_corner_graph.get_exact_distance_between_corner_indices(i, curr_index))
    if (closed[curr_index])
      continue;
    closed[curr_index] = true;

    
  }
}

void GeometricContainersOutgoing::preprocess_immediate(const Graph &graph, const CornerVector &corner_vector, const RelevantPoints &relevant_points)
{
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    for (const DivDirection outgoing_divdirection : get_divdirections())
    {
      Bound curr_bound(xyLoc(0,0), xyLoc(0,0));
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

      corner_to_outgoing_direction_to_immediate_bound[i][outgoing_divdirection] = curr_bound;
    }
  }
}

void GeometricContainersOutoing::preprocess_overall(const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points)
{
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    for (const DivDirection outgoing_divdirection : get_divdirections())
    {
      corner_to_outgoing_direction_to_bound[i][outgoing_divdirection ] = search_relevant(i,outgoing_divdirection, graph, corner_vector, complete_corner_graph, relevant_points);
    }
  }
}

void GeometricContainersOutgoing::preprocess(const Graph &graph, const CornerVector &corner_vector, const CompleteCornerGraph &complete_corner_graph, const RelevantPoints &relevant_points)
{
  preprocess_immediate(graph, corner_vector, relevant_points);
  preprocess_overall(graph, corner_vector, complete_corner_graph, relevant_points);
}


