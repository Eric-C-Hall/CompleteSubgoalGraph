#include "SimpleDijkstraExactDistance.hpp"

#include <iostream>

#include "../Graph/Graph.hpp"
#include "../Graph/Directions.hpp"

// The purpose of this class is to be a simple pathfinding implementation to
// compare against more complex pathfinding implementations to test for
// correctness

SimpleDijkstraExactDistance::SimpleDijkstraExactDistance(map_position pos, exact_distance* distances, const Graph &input_graph)
  : graph(input_graph), _distances(distances)
{
  for (unsigned int i = 0; i < graph.num_positions(); i++)
  {
    _distances[i] = MAX_EXACT_DISTANCE;
  }

  _open.push_back(pos);
  _distances[pos] = exact_distance(0,0);

  while (!_open.empty()) {
    expand_first();
  }
}

SimpleDijkstraExactDistance::~SimpleDijkstraExactDistance()
{}

void SimpleDijkstraExactDistance::expand_first()
{
  map_position first = _open.front();
  _open.pop_front();
  exact_distance current_distance = _distances[first];

  for (const Direction dir : get_directions()) {
    // Ensure no corners are being cut
    if (!is_cardinal_direction(dir))
    {
      map_position cut_corner_a = graph.step_in_direction(first, get_45_degrees_clockwise(dir));
      map_position cut_corner_b = graph.step_in_direction(first, get_45_degrees_anticlockwise(dir));
      if (graph.is_obstacle(cut_corner_a) || graph.is_obstacle(cut_corner_b))
      {
        continue;
      }
    }

    // Ensure new position is not an obstacle
    map_position other = graph.step_in_direction(first, dir);
    if (graph.is_obstacle(other)) {
      continue;
    }

    // Ensure path to new position is fastest known path
    exact_distance other_distance = current_distance + exact_distance_of_direction((Direction)dir);
    if (other_distance >= _distances[other]) {
      continue;
    }

    // Update fastest known path to other
    _distances[other] = other_distance;
    _open.push_back(other);
  }
}

void SimpleDijkstraExactDistance::run_dijkstra(map_position pos, exact_distance* distances, const Graph &graph)
{
  SimpleDijkstraExactDistance dij(pos,distances,graph);
}

// eof
