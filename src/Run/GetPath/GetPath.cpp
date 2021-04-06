#include "GetPath.hpp"

#include <vector>

bool GetPath(const PreprocessingData &preprocessing_data, xyLoc s, xyLoc g, std::vector<xyLoc> &path)
{
  s.x++;
  s.y++;
  g.x++;
  g.y++;

  map_position start = preprocessing_data.get_graph().pos(s.x, s.y);
  map_position goal = preprocessing_data.get_graph().pos(g.x, g.y);

  return preprocessing_data.get_path(start, goal, path);
}

// eof
