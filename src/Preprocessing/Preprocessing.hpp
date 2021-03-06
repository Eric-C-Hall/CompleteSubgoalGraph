#ifndef CORNERGRAPH_PROJECT__PREPROCESSING_HPP
#define CORNERGRAPH_PROJECT__PREPROCESSING_HPP

#include <vector>
#include <climits>

#include "../Graph/Graph.hpp"
#include "../Graph/XYLoc.hpp"
#include "../Graph/Directions.hpp"
#include "CornerVector.hpp"
#include "NearbyCornersWithRelevant.hpp"
#include "NearbyCornersWithNext.hpp"
#include "CompleteCornerGraph.hpp"
#include "GeometricContainers.hpp"

class PreprocessingData {
  private:
  CornerVector corner_vector;
  NearbyCornersWithRelevant nearby_corners_with_relevant;
  NearbyCornersWithNext nearby_corners_with_next;
  CompleteCornerGraph complete_corner_graph;
  GeometricContainersIncoming geometric_containers_incoming;

  void _save(std::ostream & stream, const Graph &graph) const;
  void _load(std::istream &stream, const Graph &graph);

  void _output_debug_stats() const;
  void _output_warnings() const;

  bool _try_direct_path(map_position start, map_position goal) const;

  public:
  void preprocess(const Graph &graph);
  void remove_collar(const Graph &graph); // Note: doesn't remove collar from graph
  void save(const std::string &filename, const Graph &graph) const;
  void load(const std::string &filename, const Graph &graph);

  const CornerVector &get_corner_vector() const {return corner_vector;}
  const NearbyCornersWithRelevant &get_nearby_corners_with_relevant() const {return nearby_corners_with_relevant;}
  const NearbyCornersWithNext &get_nearby_corners_with_next() const {return nearby_corners_with_next;}
  const CompleteCornerGraph &get_complete_corner_graph() const {return complete_corner_graph;}
  const GeometricContainersIncoming &get_geometric_containers_incoming() const {return geometric_containers_incoming;}
};

#endif
