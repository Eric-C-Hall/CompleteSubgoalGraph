#ifndef CORNERGRAPH_PROJECT__SIMPLE_DIJKSTRA_EXACT_DISTANCE_HPP
#define CORNERGRAPH_PROJECT__SIMPLE_DIJKSTRA_EXACT_DISTANCE_HPP

// The purpose of this class is to be a simple pathfinding implementation to
// compare against more complex pathfinding implementations to test for
// correctness

#include <deque>

#include "../Graph/ExactDistance.hpp"
#include "../Graph/Graph.hpp"

class SimpleDijkstraExactDistance {

private:
  const Graph &graph;
  std::deque<map_position> _open;
  exact_distance * _distances;

public:
  SimpleDijkstraExactDistance(map_position pos, exact_distance* distances, const Graph &graph);
  ~SimpleDijkstraExactDistance();
  static void run_dijkstra(map_position pos, exact_distance* distances, const Graph &graph);

private:
  void expand_first();
};

#endif
