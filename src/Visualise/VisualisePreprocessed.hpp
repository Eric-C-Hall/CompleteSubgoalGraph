#ifndef CORNERGRAPH_PROJECT__VISUALISE_PREPROCESSED_HPP
#define CORNERGRAPH_PROJECT__VISUALISE_PREPROCESSED_HPP

#include <vector>

#include "../Preprocess/Preprocess.hpp"

void Visualise(const PreprocessingData &preprocessing_data);

void print_graph(const Graph &graph, const std::vector<map_position> &cursors, const std::vector<xyLoc> &path);

#endif
