#ifndef CORNERGRAPH_PROJECT__VISUALISE_PREPROCESSED_HPP
#define CORNERGRAPH_PROJECT__VISUALISE_PREPROCESSED_HPP

#include <vector>

#include "../Preprocess/Preprocess.hpp"

void Visualise(const PreprocessingData &preprocessing_data);

void print_graph(const PreprocessingData &preprocessing_data, const Graph &graph, const std::vector<map_position> &cursors, const std::vector<xyLoc> &path, const bool show_nearby, const bool show_num_nearby, const bool show_num_nearby_with_next, const bool show_nearby_with_next, const bool show_islands, const bool show_bounds, const MidDirection middirection, const unsigned int which_nearby_corner);

#endif
