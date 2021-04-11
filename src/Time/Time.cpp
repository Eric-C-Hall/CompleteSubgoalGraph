#include <random>

#include "Time.hpp"
#include "../Utility/Timer.h"
#include "../Run/GetPath/GetPath.hpp"

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

void Time(const PreprocessingData &preprocessing_data, unsigned int num_tests)
{
  const Graph &graph = preprocessing_data.get_graph();
  const unsigned int width = graph.get_width();
  const unsigned int height = graph.get_height();

  // A vector of pairs of locations, to calculate the distance from one to the other.
  std::vector<xyLoc> testLocs;
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
    // TODO: Perhaps check if these points are on the same island
    testLocs.push_back(sourceLoc);
    testLocs.push_back(destLoc);
  }

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



  // Output time

  std::cout << "Total time: " << t.GetElapsedTime() << "\n";
  std::cout << "Num tests: " << num_tests << "\n";
  std::cout << "Time per test: " << t.GetElapsedTime() / num_tests << "\n";

  std::cout << std::flush;

}
