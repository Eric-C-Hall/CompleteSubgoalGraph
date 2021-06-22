#ifndef CORNERGRAPH_PROJECT__VISUALISE_PREPROCESSED_HPP
#define CORNERGRAPH_PROJECT__VISUALISE_PREPROCESSED_HPP

#include <vector>

#include "../Preprocess/Preprocess.hpp"

void Visualise(const PreprocessingData &preprocessing_data);

struct PrintGraphArguments
{
  bool show_nearby = false;
  bool show_num_nearby = false;
  bool show_num_nearby_with_next = false;
  bool show_nearby_with_next = false;
  bool show_islands = false;
  bool show_bounds = false;
  MidDirection middirection = Dir_NNE;
  unsigned int which_nearby_corner = 0;
  int selected_island = -1;
};

void print_graph(const PreprocessingData &preprocessing_data, const Graph &graph, const std::vector<map_position> &cursors, const std::vector<xyLoc> &path, const PrintGraphArguments &print_graph_arguments);

char int_to_drawn_char(int i);

#endif
