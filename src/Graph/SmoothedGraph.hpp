#ifndef CORNERGRAPH_PROJECT__SMOOTHED_GRAPH_HPP
#define CORNERGRAPH_PROJECT__SMOOTHED_GRAPH_HPP

#include "Graph.hpp"
#include "Corners.hpp"

struct SmoothedGraphPrintArgs
{
  corner_index selected_corner = 0;
  map_position selected_position = -1;
};

class SmoothedGraph
{
  private:
  const Graph &graph;

  // TODO: This should be constant, but currently cannot be initialised easily as constant
  unsigned int initial_num_free_spaces;
  std::vector<map_position> _initial_corners;
  std::vector<corner_index> _pos_to_initial_corner_index;

  std::vector<bool> _obstacles;

  // When modifying the graph, keep track of the most recently added obstacles and removable corners,
  // so as to 
  std::vector<map_position> last_added_obstacles;
  std::set<map_position> removable_corners;

  std::vector<map_position> _corners;
  std::vector<corner_index> _pos_to_corner_index;

  // ---------------------
  // TODO: This is a copy of stuff from PreprocessingData. Perhaps this and the stuff from PreprocessingData should be combined somehow
  bool _is_corner(map_position p);
  void _add_if_corner(map_position p);
  void _compute_corners();
  // ---------------------

  unsigned int compute_num_free_spaces() const;

  void print(const SmoothedGraphPrintArgs &args) const;

  // ------------------------------------------------------
  // Automatically smoothen the graph in 3 steps:
  void auto_smoothen_straight();
  // Step 1: Add obstacles
  void smoothen_straight(const map_position c);
  Direction find_wall_direction(const map_position c) const;
  unsigned int create_obstacles_in_direction(map_position wall, map_position corner, Direction dir);
  void flood_fill_obstacles_in_direction(map_position origin, Direction dir, unsigned int dist);
  void flood_fill_obstacles(map_position origin);

  void add_obstacle(map_position p);
  void add_obstacles(const std::vector<map_position> &new_obstacles);
  // Step 2: Check effect of adding obstacles on corners
  // Note: we don't take into account the possibility of corners needing to be added,
  // because this will never occur in the current algorithm
  void find_newly_removable_corners();
  bool smoothen_acceptable() const;
  // Step 3: Confirm/Revert
  void undo_add_obstacles();
  void confirm_add_obstacles();
  void remove_obstacles(const std::vector<map_position> &remove_obstacles);
  // ------------------------------------------------------



  // ------------------------------------------------------
  // Smoothen diagonal
  void auto_smoothen_diagonal();
  void smoothen_diagonal(const map_position c, const bool use_clockwise_diagonal);
  unsigned int create_obstacles_along_diagonal(map_position first, Direction dir1, Direction dir2);
  // ------------------------------------------------------

  public:
  SmoothedGraph(const Graph &input_graph);
  void gui();

  
};



#endif
