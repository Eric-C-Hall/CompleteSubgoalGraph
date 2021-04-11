#include "Test.hpp"

#include "../Graph/ExactDistance.hpp"
#include "../Visualise/VisualisePreprocessed.hpp"
#include "../Run/GetPath/GetPath.hpp"
#include "../Graph/ValidatePath.hpp"
#include "SimpleDijkstraExactDistance.hpp"

#include <random>

void Test(const PreprocessingData &preprocessing_data)
{
  const static bool TEST_SPECIFIC_POINTS = false;
  const static int TEST_SPECIFIC_POINTS_A_X = 86;//(113, 331). Target: (252, 211)
  const static int TEST_SPECIFIC_POINTS_A_Y = 262;
  const static int TEST_SPECIFIC_POINTS_B_X = 99;
  const static int TEST_SPECIFIC_POINTS_B_Y = 264;

  const Graph &graph = preprocessing_data.get_graph();
  const unsigned int width = graph.get_width();
  const unsigned int height = graph.get_height();

  int x1 = 0, y1 = 0, x2, y2;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> width_distr(0, width-1);
  std::uniform_int_distribution<> height_distr(0, height-1);

  exact_distance * dijkstra_dists = NULL;

  for (int i = 0;; i++)
  {
    if ((i & 0b1111111111) == 0)
    {
      std::cout << "Starting " << i <<"th iteration" << std::endl;
      // Randomly choose x1 and y1;
      do
      {
        x1 = width_distr(gen);
        y1 = height_distr(gen);
      }
      while (graph.is_obstacle(graph.pos(x1, y1)));

      if (TEST_SPECIFIC_POINTS)
      {
        std::cout << "testing the specific points (" << TEST_SPECIFIC_POINTS_A_X << ", " << TEST_SPECIFIC_POINTS_A_Y << ") and (" << TEST_SPECIFIC_POINTS_B_X << ", " << TEST_SPECIFIC_POINTS_B_Y << ")" << std::endl;
        x1 = TEST_SPECIFIC_POINTS_A_X;
        y1 = TEST_SPECIFIC_POINTS_A_Y;
      }

      // Use SimpleDijkstra to find the distance of this point from all others.
      delete[] dijkstra_dists;
      dijkstra_dists = new exact_distance[width * height];
      SimpleDijkstraExactDistance::run_dijkstra(graph.pos(x1,y1), dijkstra_dists, graph);
    }

    //randomly choose x2 and y2
    do
    {
      x2 = width_distr(gen);
      y2 = height_distr(gen);
    }
    while (graph.is_obstacle(graph.pos(x2, y2)) || dijkstra_dists[graph.pos(x2, y2)] == MAX_EXACT_DISTANCE);

    if (TEST_SPECIFIC_POINTS)
    {
      x2 = TEST_SPECIFIC_POINTS_B_X;
      y2 = TEST_SPECIFIC_POINTS_B_Y;
    }

    // find a path between (x1,y1) and (x2,y2) using corner graphs
    std::vector<xyLoc> path;
    GetPath(preprocessing_data, xyLoc(x1, y1), xyLoc(x2, y2), path);

    if (path[0].x != x1 ||
        path[0].y != y1 ||
        path.back().x != x2 ||
        path.back().y != y2)
    {
       throw std::runtime_error("path does not match experiment. " + std::to_string(path[0].x) + " vs " + std::to_string(x1) + ", " + std::to_string(path[0].y) + " vs " + std::to_string(y1) + ", " + std::to_string(path.back().x) + " vs " + std::to_string(x2) + ", " + std::to_string(path.back().y) + " vs " + std::to_string(y2));
    }

    // Validate path
    if (!ValidatePath(graph, path))
    {
      print_graph(graph, std::vector<map_position>(), path);
      throw std::runtime_error("Failed to validate path returned from main. Source: ("  + std::to_string(x1) + ", " + std::to_string(y1) + "). Target: (" + std::to_string(x2) + ", " + std::to_string(y2) + ").");
    }

    // test if this path has the same length as SimpleDijkstra

    exact_distance path_length = ZERO_EXACT_DISTANCE;

    // Find num_straight and num_diagonal for path found by brigitte
    for (unsigned int i = 1; i < path.size(); i++)
    {
      const xyLoc &prev = path[i-1];
      const xyLoc &curr = path[i];
      if (std::abs(prev.x - curr.x) == 1 && std::abs(prev.y - curr.y) == 1)
        path_length.num_diagonal++;
      else
        path_length.num_straight++;
    }

    // Check that SimpleDijkstra and Brigitte return the same length
    exact_distance simple_path_length = dijkstra_dists[graph.pos(x2, y2)];
    if (path_length != simple_path_length)
    {
      print_graph(graph, std::vector<map_position>(), path);
      throw std::runtime_error("Test failed from (" + std::to_string(x1) + ", " + std::to_string(y1) + ") to (" + std::to_string(x2) + ", " + std::to_string(y2) + "). Simple had " + std::to_string(simple_path_length.num_straight) + " straight, " + std::to_string(simple_path_length.num_diagonal) + " diagonal. Main had " + std::to_string(path_length.num_straight) + " straight, " + std::to_string(path_length.num_diagonal) + " diagonal.");
    }
  }
  delete[] dijkstra_dists;
}
