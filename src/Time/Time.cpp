#include <random>

#include "Time.hpp"
#include "../Utility/Timer.h"
#include "../Run/GetPath/GetPath.hpp"
#include "../Graph/Islands.hpp"

xyLoc GetRandomValidLocation(std::mt19937 &gen,
                             std::uniform_int_distribution<> &width_distr,
                             std::uniform_int_distribution<> &height_distr,
                             const Graph &graph)
{
  xyLoc returnLoc;
  do
  {
    returnLoc.x = width_distr(gen);
    returnLoc.y = height_distr(gen);
  }
  while (graph.is_obstacle(graph.pos(returnLoc)));

  return returnLoc;
}

void get_test_locs(std::vector<xyLoc> &testLocs, const unsigned int num_tests, const Graph &graph, const Islands &islands)
{
  const unsigned int width = graph.get_width();
  const unsigned int height = graph.get_height();

  testLocs.clear();
  testLocs.reserve(2 * num_tests);

  // RNG stuff
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> width_distr(0, width-1);
  std::uniform_int_distribution<> height_distr(0, height-1);

  // Create random locations to test distance between
  for (unsigned int i = 0; i < num_tests; i++)
  {
    xyLoc sourceLoc = GetRandomValidLocation(gen, width_distr, height_distr, graph);
    xyLoc destLoc = GetRandomValidLocation(gen, width_distr, height_distr, graph);
    // Check if these points are on the same island
    if (islands.get_island_index(graph.pos(sourceLoc)) != islands.get_island_index(graph.pos(destLoc)))
    {
      continue;
    }
    testLocs.push_back(sourceLoc);
    testLocs.push_back(destLoc);
  }
}

void Time(const PreprocessingData &preprocessing_data, unsigned int num_tests)
{
  const Graph &graph = preprocessing_data.get_graph();
  const CornerVector &corner_vector = preprocessing_data.get_corner_vector();
  const Islands islands(graph);

  // A vector of pairs of locations, to calculate the distance from one to the other.
  std::vector<xyLoc> testLocs;
  get_test_locs(testLocs, num_tests, graph, islands);

  // Setup for timed portion
  auto testLocIter = testLocs.begin();
  std::vector<xyLoc> path;
  Timer t;

  // ---------------
  // Timed code starts

  t.StartTimer();

  while (testLocIter != testLocs.end())
  {
    xyLoc sourceLoc = *testLocIter;
    testLocIter++;
    xyLoc destLoc = *testLocIter;
    testLocIter++;
    
    GetPath(preprocessing_data, sourceLoc, destLoc, path);
  }

  t.EndTimer();

  // Timed code ends
  // ---------------

  get_test_locs(testLocs, num_tests, graph, islands);
  testLocIter = testLocs.begin();
  Timer octile_t;
  // ---------------
  // Timed code starts

  octile_t.StartTimer();

  while (testLocIter != testLocs.end())
  {
    xyLoc sourceLoc = *testLocIter;
    testLocIter++;
    xyLoc destLoc = *testLocIter;
    testLocIter++;
    
    Running::get_path_octile_step_only(graph.pos(sourceLoc), graph.pos(destLoc), path, graph, corner_vector);
  }

  octile_t.EndTimer();

  // Timed code ends
  // ---------------

  get_test_locs(testLocs, num_tests, graph, islands);
  testLocIter = testLocs.begin();
  Timer double_t;
  // ---------------
  // Timed code starts

  double_t.StartTimer();

  while (testLocIter != testLocs.end())
  {
    xyLoc sourceLoc = *testLocIter;
    testLocIter++;
    xyLoc destLoc = *testLocIter;
    testLocIter++;
    
    Running::get_path_up_to_double_step(graph.pos(sourceLoc), graph.pos(destLoc), path, graph, corner_vector);
  }

  double_t.EndTimer();

  // Timed code ends
  // ---------------

  get_test_locs(testLocs, num_tests, graph, islands);
  testLocIter = testLocs.begin();
  Timer single_t;
  // ---------------
  // Timed code starts

  single_t.StartTimer();

  while (testLocIter != testLocs.end())
  {
    xyLoc sourceLoc = *testLocIter;
    testLocIter++;
    xyLoc destLoc = *testLocIter;
    testLocIter++;
    
    Running::get_path_up_to_single_step(graph.pos(sourceLoc), graph.pos(destLoc), path, graph, corner_vector);
  }

  single_t.EndTimer();

  // Timed code ends
  // ---------------

  // Output time

  double time_per_test = t.GetElapsedTime() / num_tests;
  std::cout << "Total time: " << t.GetElapsedTime() << "\n";
  std::cout << "Num tests: " << num_tests << "\n";
  std::cout << "Time per test: " << time_per_test << "\n";
  std::cout << std::endl;

  double time_per_octile_step = octile_t.GetElapsedTime() / num_tests;
  double time_per_double_step = (double_t.GetElapsedTime() / num_tests) - (octile_t.GetElapsedTime() / num_tests);
  double time_per_single_step = (single_t.GetElapsedTime() / num_tests) - (double_t.GetElapsedTime() / num_tests);
  double time_per_compute_step = (t.GetElapsedTime() / num_tests) - (single_t.GetElapsedTime() / num_tests);
  std::cout << "Time per step:" << "\n";
  std::cout << "Octile  step: " << time_per_octile_step << "\n";
  std::cout << "Double  step: " << time_per_double_step << "\n";
  std::cout << "Single  step: " << time_per_single_step << "\n";
  std::cout << "Compute step: " << time_per_compute_step << "\n";
  std::cout << std::endl;

  std::cout << "Percentage of time per step:" << "\n";
  std::cout << "Octile  step: " << 100 * time_per_octile_step / time_per_test << "\n";
  std::cout << "Double  step: " << 100 * time_per_double_step / time_per_test << "\n";
  std::cout << "Single  step: " << 100 * time_per_single_step / time_per_test << "\n";
  std::cout << "Compute step: " << 100 * time_per_compute_step / time_per_test << "\n";
  std::cout << std::flush;

}
