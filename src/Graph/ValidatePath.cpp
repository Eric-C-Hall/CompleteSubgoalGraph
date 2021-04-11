#include "ValidatePath.hpp"

bool ValidatePath(const Graph &graph, const std::vector<xyLoc> &path)
{
  for (int i = 0; i < (int)path.size()-1; i++)
  {
    if (abs(path[i].x - path[i+1].x) > 1)
      return false;
    if (abs(path[i].y - path[i+1].y) > 1)
      return false;
    if (graph.is_obstacle(graph.pos(path[i].x, path[i].y)))
      return false;
    if (graph.is_obstacle(graph.pos(path[i+1].x, path[i+1].y)))
      return false;
    // I believe the following disallows cutting corners, whereas we allow cutting corners.
    //
    //if (path[i].x != path[i+1].x && path[i].y != path[i+1].y)
    //{
    //  if (graph.is_obstacle(graph.pos(path[i+1].x, path[i].y)))
    //    return false;
    //  if (graph.is_obstacle(graph.pos(path[i].x, path[i+1].y)))
    //    return false;
    //}
  }
  return true;
}
