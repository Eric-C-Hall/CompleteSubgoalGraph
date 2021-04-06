#include <algorithm>
#include <numeric>

#include "stats.hpp"

double stats::GetTotalTime()
{
  return std::accumulate(times.begin(), times.end(), 0.0);
}

double stats::GetMaxTimestep()
{
  return *std::max_element(times.begin(), times.end());
}

double stats::Get20MoveTime()
{
  for (unsigned int x = 0; x < lengths.size(); x++)
    if (lengths[x] >= 20)
      return std::accumulate(times.begin(), times.begin()+1+x, 0.0);
  return GetTotalTime();
}

double stats::GetPathLength()
{
  double len = 0;
  for (int x = 0; x < (int)path.size()-1; x++)
  {
    if (path[x].x == path[x+1].x || path[x].y == path[x+1].y)
    {
      len++;
    }
    else {
      len += 1.4142;
    }
  }
  return len;
}

bool stats::ValidatePath(int width, int height, const Graph &graph)
{
  for (int x = 0; x < (int)path.size()-1; x++)
  {
    if (abs(path[x].x - path[x+1].x) > 1)
      return false;
    if (abs(path[x].y - path[x+1].y) > 1)
      return false;
    if (graph.is_obstacle(graph.pos(path[x].x, path[x].y)))
      return false;
    if (graph.is_obstacle(graph.pos(path[x+1].x, path[x+1].y)))
    {
      if (PRINT_DEBUG_PATH_IF_FAILS_TO_VALIDATE)
      {
        //debug_print_path(width, height, mapData);
      }
      return false;
    }
    // I believe the following disallows cutting corners, whereas we allow cutting corners.
    //
    //if (path[x].x != path[x+1].x && path[x].y != path[x+1].y)
    //{
    //  if (!mapData[path[x+1].y*width+path[x].x])
    //    return false;
    //  if (!mapData[path[x].y*width+path[x+1].x])
    //    return false;
    //}
  }
  return true;
}

/*void stats::debug_print_path(int width, int height, const std::vector<bool>&mapData)
{
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      xyLoc currLoc(x,y);
      const auto currLoc_first_iter_in_path = std::find(path.begin(), path.end(), currLoc);

      if (!mapData[y*width + x])
      {
        if (currLoc_first_iter_in_path != path.end())
        {
          std::cout << "!";
        }
        else
        {
          std::cout << "@";
        }
      }
      else if (currLoc_first_iter_in_path != path.end())
      {
        const auto currLoc_second_iter_in_path = std::find(currLoc_first_iter_in_path + 1, path.end(), currLoc);

        if (currLoc_second_iter_in_path != path.end())
        {
          std::cout << "O";
        }
        else if (currLoc_first_iter_in_path == path.begin())
        {
          std::cout << "A";
        }
        else if (currLoc_first_iter_in_path + 1 == path.end())
        {
          std::cout << "B";
        }
        else
        {
          std::cout << "X";
        }
      }
      else
      {
        std::cout << " ";
      }
    }
    std::cout << "\n";
  }
}*/

