#ifndef CORNERGRAPH_PROJECT__TEST_HPP
#define CORNERGRAPH_PROJECT__TEST_HPP

#include "../Preprocessing/Preprocessing.hpp"
#include "../Graph/ExactDistance.hpp"
#include "../Graph/Islands.hpp"

#include <random>

class TestPointGenerator
{
  private:
  const bool ONLY_TEST_NEARBY_POINTS = false;
  const exact_distance NEARBY_TEST_DISTANCE = 10 * STRAIGHT_EXACT_DISTANCE;

  const bool TEST_SPECIFIC_POINTS = false;
  const int TEST_SPECIFIC_POINTS_A_X = 183;//(113, 331). Target: (252, 211)
  const int TEST_SPECIFIC_POINTS_A_Y = 61;
  const int TEST_SPECIFIC_POINTS_B_X = 139;
  const int TEST_SPECIFIC_POINTS_B_Y = 152;

  const Graph &graph;
  const Islands islands;

  std::random_device rd;
  std::mt19937 gen;
  std::uniform_int_distribution<> width_distr;
  std::uniform_int_distribution<> height_distr;

  xyLoc get_random_first_point();
  xyLoc get_random_other_point(xyLoc first_point);
  void print_warnings() const;

  public:
  TestPointGenerator(const Graph &input_graph) : graph(input_graph), islands(input_graph), gen(rd()), width_distr(0, input_graph.get_width()-1), height_distr(0, input_graph.get_height()-1) {}
  std::pair<xyLoc,std::vector<xyLoc>> get_random_point_and_points(int num_points);
  std::vector<std::pair<xyLoc, xyLoc>> get_random_pairs_of_points(int num_points);
  std::vector<xyLoc> get_random_other_points(xyLoc first_point, int num_points);

};

void Test(const PreprocessingData &preprocessing_data, const Graph &graph);

#endif
