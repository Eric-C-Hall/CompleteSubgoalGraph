#ifndef CORNERGRAPH_PROJECT__VISUALISE_PREPROCESSED_HPP
#define CORNERGRAPH_PROJECT__VISUALISE_PREPROCESSED_HPP

#include <vector>

#include "../Preprocessing/Preprocessing.hpp"

void Visualise(const PreprocessingData &preprocessing_data);

struct PrintGraphArguments
{
  bool show_nearby = false;
  bool show_bounds = false;
  MidDirection middirection = Dir_NNE;
  bool show_divdirection = false;
  DivDirection divdirection = DivDir_N;
};

void print_graph(const PreprocessingData &preprocessing_data, const std::vector<map_position> &cursors, const std::vector<xyLoc> &path, const PrintGraphArguments &print_graph_arguments);

#endif
