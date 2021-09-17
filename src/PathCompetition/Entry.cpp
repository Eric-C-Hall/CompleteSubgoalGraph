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
PreprocessingData preprocessing_data;

void PreprocessMap(std::vector<bool> &bits, int width, int height, const char *filename)
{
  graph.load_bits_with_collar(bits, (unsigned int)width, (unsigned int)height);
  preprocessing_data.preprocess(graph);
  preprocessing_data.remove_collar(graph);
  graph.remove_collar();
  preprocessing_data.save(filename, graph);
}

void *PrepareForSearch(std::vector<bool> &bits, int w, int h, const char *filename)
{
  graph.load_bits_without_collar(bits, (unsigned int)w, (unsigned int)h);
  preprocessing_data.load(filename, graph);
}

bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path)
{
  map_position start = graph.pos(s.x, s.y);
  map_position goal = graph.pos(g.x, g.y);

  Running::get_path(start, goal, path, preprocessing_data, graph);

  return true;
}

