#ifndef CORNERGRAPH_PROJECT__GET_PATH_HPP
#define CORNERGRAPH_PROJECT__GET_PATH_HPP

#include "../../Preprocessing/Preprocessing.hpp"
#include "../../Graph/XYLoc.hpp"

bool GetPath(const PreprocessingData &preprocessing_data, xyLoc s, xyLoc g, std::vector<xyLoc> &path);

  // ------------------------
  // Implemented in GetPath.cpp
  private:
  template<bool try_octile, bool test_double, bool test_single, bool compute_path>
  bool get_path_partial_computation(map_position start, map_position goal, std::vector<xyLoc> &path) const;
  public:
  bool get_path_octile_step_only(map_position start, map_position goal, std::vector<xyLoc> &path) const;
  bool get_path_up_to_double_step(map_position start, map_position goal, std::vector<xyLoc> &path) const;
  bool get_path_up_to_single_step(map_position start, map_position goal, std::vector<xyLoc> &path) const;
  bool get_path(map_position start, map_position goal, std::vector<xyLoc> &path) const;
  //-------------------------

#endif
