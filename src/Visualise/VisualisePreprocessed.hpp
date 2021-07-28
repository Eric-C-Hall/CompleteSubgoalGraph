#ifndef CORNERGRAPH_PROJECT__VISUALISE_PREPROCESSED_HPP
#define CORNERGRAPH_PROJECT__VISUALISE_PREPROCESSED_HPP

#include <vector>

#include "../Preprocessing/Preprocessing.hpp"

void Visualise(const PreprocessingData &preprocessing_data);

struct PrintGraphArguments
{
  bool show_corners = false;
  bool show_nearby = false;
  bool show_nearby_with_relevant = false;
  bool show_nearby_with_next = false;
  bool show_outgoing_bounds = false;
  bool show_incoming_bounds = false;
  bool show_relevant_corners = false;
  bool show_relevant_divdirections = false;
  bool show_relevant_points = false;
  bool show_incoming_to_relevant_points = false;
  bool show_divdirection = false;
  DivDirection divdirection = DivDir_N;
  int which_nearby_corner = -1;
};

void print_graph(const PreprocessingData &preprocessing_data, const std::vector<map_position> &cursors, const std::vector<xyLoc> &path, const PrintGraphArguments &print_graph_arguments);

#endif
