#ifndef CORNERGRAPH_PROJECT__SAFE_REACHABLE_HPP
#define CORNERGRAPH_PROJECT__SAFE_REACHABLE_HPP

#include <vector>

#include "../Graph/MapPosition.hpp"
#include "../Graph/Directions.hpp"
#include "../Graph/Graph.hpp"

namespace Preprocessing
{
  void get_safe_reachable_in_directions(const Graph &graph, std::vector<map_position> &output, const map_position origin, const Direction straight_direction, const Direction diagonal_direction, const int num_step_bound = INT_MAX);
  std::vector<map_position> get_safe_reachable_in_all_directions(const Graph &graph, const map_position origin);

}

#endif
