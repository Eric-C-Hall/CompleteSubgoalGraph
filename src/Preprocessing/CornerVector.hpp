#ifndef CORNERGRAPH_PROJECT__CORNER_VECTOR_HPP
#define CORNERGRAPH_PROJECT__CORNER_VECTOR_HPP

#include <limits>
#include "../Graph/Graph.hpp"
#include "../Graph/MapPosition.hpp"

typedef uint16_t corner_index;

#define MAX_POSSIBLE_CORNER_INDEX (std::numeric_limits<uint16_t>::max())

class CornerVector
{
  private:
  std::vector<map_position> corners;

  bool calculate_is_corner(map_position p, const Graph &graph);

  public:
  void preprocess(const Graph &graph);
  void save(std::ostream &stream) const;
  void load(std::istream &stream);

  map_position get_corner(int i) const {return corners[i];}
  size_t size() const {return corners.size();}
};

#endif
