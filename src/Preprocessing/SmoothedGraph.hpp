#ifndef CORNERGRAPH_PROJECT__SMOOTHED_GRAPH_HPP
#define CORNERGRAPH_PROJECT__SMOOTHED_GRAPH_HPP

#include <vector>
#include <set>

#include "../Graph/MapPosition.hpp"
#include "../Graph/Graph.hpp"
#include "CornerVector.hpp"

class SmoothedGraph
{
  private:
  unsigned int MAX_FLOOD_FILL_BOUNDARY_SIZE = 1000;

  std::vector<bool> is_blocked;
  std::vector<bool> is_corner;

  // When modifying the graph, keep track of the most recently added obstacles and removable corners,
  // so as to be able to undo unwanted changes
  std::vector<map_position> last_added_obstacles;
  std::set<map_position> removable_corners;
  std::set<map_position> addable_corners;

  // TODO: this is basically copy/pasted from CornerVector.hpp, perhaps this can be done in a more nice way
  bool calculate_is_corner(map_position p, const Graph &graph);
  std::vector<map_position> calculate_corners(const Graph &graph);

  // ------------------------------------------------------
  // Automatically smoothen the graph in 3 steps:
  void auto_smoothen_straight(const Graph &graph, const CornerVector &corner_vector);
  // Step 1: Add obstacles
  void smoothen_straight(const map_position c, const Direction diagonal_direction, const bool go_clockwise, const Graph &graph);
  unsigned int create_obstacles_in_direction(map_position construct_wall_pos, map_position before_wall_pos, Direction construct_direction, const Graph &graph);
  bool flood_fill_obstacles_in_direction(map_position origin, Direction dir, unsigned int dist, const Graph &graph);
  bool flood_fill_obstacles(const map_position origin, const Graph &graph);

  void add_obstacle(map_position p, const Graph &graph);
  void add_obstacles(const std::vector<map_position> &new_obstacles, const Graph &graph);
  // Step 2: Check effect of adding obstacles on corners
  // Note: we don't take into account the possibility of corners needing to be added,
  // because this will never occur in the current algorithm
  void find_newly_removable_corners(const Graph &graph);
  bool smoothen_acceptable() const;
  // Step 3: Confirm/Revert
  void undo_add_obstacles();
  void confirm_add_obstacles(const Graph &graph);
  void remove_obstacles(const std::vector<map_position> &remove_obstacles);
  // ------------------------------------------------------

  // ------------------------------------------------------
  // Smoothen diagonal
  void auto_smoothen_diagonal();
  void smoothen_diagonal(const map_position c, const bool use_clockwise_diagonal);
  unsigned int create_obstacles_along_diagonal(map_position first, Direction dir1, Direction dir2);
  // ------------------------------------------------------

  public:
  void preprocess(const Graph &graph, const CornerVector &corner_vector);

  void print(Printer &printer, const Graph &graph) const;
  void print(const Graph &graph) const;
};



#endif
