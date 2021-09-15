#include <deque>
#include <vector>
#include <algorithm>
#include <assert.h>
#include "Entry.h"

#include <memory>

#include "../Graph/Graph.hpp"
#include "../Preprocessing/Preprocessing.hpp"
#include "../Run/GetPath.hpp"

void GetSuccessors(xyLoc s, std::vector<xyLoc> &neighbors);
int GetIndex(xyLoc s);
void ExtractPath(xyLoc end, std::vector<xyLoc> &finalPath);

const char *GetName()
{
  return "CompleteCornerGraph";
}

Graph graph;
PreprocessingData preprocessing_data(graph);

void PreprocessMap(std::vector<bool> &bits, int width, int height, const char *filename)
{
  graph.load_bits(bits, (unsigned int)width, (unsigned int)height);
  preprocessing_data.preprocess();
  preprocessing_data.save(filename);
}

void *PrepareForSearch(std::vector<bool> &bits, int w, int h, const char *filename)
{
  graph.load_bits(bits, (unsigned int)w, (unsigned int)h);
  preprocessing_data.load(filename);
}

bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path)
{
  map_position start = graph.pos(s.x+1, s.y+1);
  map_position goal = graph.pos(g.x+1, g.y+1);

  Running::get_path(start, goal, path, preprocessing_data);

  for (xyLoc &l : path)
  {
    l.x--;
    l.y--;
  }

  return true;
}

