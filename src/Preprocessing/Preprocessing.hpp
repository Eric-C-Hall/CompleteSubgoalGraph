#ifndef CORNERGRAPH_PROJECT__PREPROCESSING_HPP
#define CORNERGRAPH_PROJECT__PREPROCESSING_HPP

#include <vector>
#include <climits>

#include "../Graph/Graph.hpp"
#include "../Graph/XYLoc.hpp"
#include "../Graph/Directions.hpp"
#include "CornerVector.hpp"
#include "NearbyCorners.hpp"
#include "NearbyCornersWithNext.hpp"
#include "CompleteCornerGraph.hpp"
#include "GeometricContainers.hpp"

void PreprocessMap(const Graph &graph);

class PreprocessingData {
  private:
  const Graph &graph;

  CornerVector corner_vector;
  NearbyCorners nearby_corners;
  NearbyCornersWithNext nearby_corners_with_next;
  CompleteCornerGraph complete_corner_graph;
  GeometricContainersIncoming geometric_containers_incoming;

  void _save(std::ostream & stream) const;
  void _load(std::istream &stream);

  void _output_debug_stats() const;
  void _output_warnings() const;

  bool _try_direct_path(map_position start, map_position goal) const;

  public:
  PreprocessingData(const Graph &graph);

  void preprocess();
  void save(const std::string &filename) const;
  void load(const std::string &filename);

  const Graph &get_graph() const {return graph;}
  const CornerVector &get_corner_vector() const {return corner_vector;}
  const NearbyCorners &get_nearby_corners() const {return nearby_corners;}
  const CompleteCornerGraph &get_complete_corner_graph() const {return complete_corner_graph;}
  const GeometricContainersIncoming &get_geometric_containers_incoming() const {return geometric_containers_incoming;}
};

#endif
