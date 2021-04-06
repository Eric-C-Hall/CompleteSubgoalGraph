#ifndef CORNERGRAPH_PROJECT__ENTRY_H
#define CORNERGRAPH_PROJECT__ENTRY_H

#include "../../Preprocess/Preprocess.hpp"
#include "../../Graph/XYLoc.hpp"

bool GetPath(const PreprocessingData &preprocessing_data, xyLoc s, xyLoc g, std::vector<xyLoc> &path);

#endif
