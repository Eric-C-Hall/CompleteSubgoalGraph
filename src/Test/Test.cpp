#include "Test.hpp"

#include "../Graph/ExactDistance.hpp"
#include "../Visualise/VisualisePreprocessed.hpp"
#include "../Run/GetPath/GetPath.hpp"
#include "../Graph/ValidatePath.hpp"
#include "SimpleDijkstraExactDistance.hpp"

xyLoc TestPointGenerator::get_random_first_point()
{
  xyLoc return_value;

  do
  {
    return_value.x = width_distr(gen);
    return_value.y = height_distr(gen);
  }
  while (graph.is_obstacle(graph.pos(return_value)));

  return return_value;
}

xyLoc TestPointGenerator::get_random_other_point(xyLoc first_point)
{
  xyLoc return_value;

  exact_distance dist;
  do
  {
    return_value.x = width_distr(gen);
    return_value.y = height_distr(gen);
    dist = graph.octile_distance(graph.pos(first_point), graph.pos(return_value));
  }
  while (graph.is_obstacle(graph.pos(return_value)) // Don't choose obstacles
         || islands.get_island_index(return_value) != islands.get_island_index(first_point) // Don't choose points in different islands
         || (ONLY_TEST_NEARBY_POINTS && dist > NEARBY_TEST_DISTANCE) // Don't choose points that are too far away (since debugging is easier when the chosen points are near each other, it has a simpler path)
        );

  return return_value;
}

void TestPointGenerator::print_warnings() const
{
  if (TEST_SPECIFIC_POINTS)
  {
    std::cout << "testing the specific points (" << TEST_SPECIFIC_POINTS_A_X << ", " << TEST_SPECIFIC_POINTS_A_Y << ") and (" << TEST_SPECIFIC_POINTS_B_X << ", " << TEST_SPECIFIC_POINTS_B_Y << ")" << std::endl;
  }

  if (ONLY_TEST_NEARBY_POINTS)
  {
    std::cout << "WARNING: ONLY TESTING NEARBY POINTS" << std::endl;
  }
}

std::pair<xyLoc,std::vector<xyLoc>> TestPointGenerator::get_random_point_and_points(int num_points)
{
  print_warnings();

  xyLoc loc1 = get_random_first_point();
  std::vector<xyLoc> other_locs;

  for (int i = 0; i < num_points; i++)
  {
    other_locs.push_back(get_random_other_point(loc1));
  }

  if (TEST_SPECIFIC_POINTS)
  {
    xyLoc specific_point_a(TEST_SPECIFIC_POINTS_A_X, TEST_SPECIFIC_POINTS_A_Y);
    std::vector<xyLoc> specific_point_b;
    specific_point_b.push_back(xyLoc(TEST_SPECIFIC_POINTS_B_X, TEST_SPECIFIC_POINTS_B_Y));
    return std::pair<xyLoc,std::vector<xyLoc>>(specific_point_a, specific_point_b);
  }

  return std::pair<xyLoc,std::vector<xyLoc>>(loc1, other_locs);
}

std::vector<std::pair<xyLoc, xyLoc>> TestPointGenerator::get_random_pairs_of_points(int num_points)
{
  // TODO: doesn't currently test specific points even if TEST_SPECIFIC_POINTS is enabled.
  print_warnings();

  std::vector<std::pair<xyLoc, xyLoc>> return_value;
  for (int i = 0; i < num_points; i++)
  {
    std::pair<xyLoc, xyLoc> curr_pair;
    curr_pair.first = get_random_first_point();
    curr_pair.second = get_random_other_point(curr_pair.first);
    return_value.push_back(curr_pair);
  }
  return return_value;
}

void Test(const PreprocessingData &preprocessing_data)
{
  const Graph &graph = preprocessing_data.get_graph();
  const unsigned int width = graph.get_width();
  const unsigned int height = graph.get_height();
  TestPointGenerator test_point_generator(graph);

  const static int NUM_TESTS_PER_ITERATION = 1000;

  exact_distance * dijkstra_dists = NULL;

  for (int i = 0;; i++)
  {
    if (i % 100 == 0)
    {
      std::cout << "Finding paths from " << i <<"th randomly chosen point to " << NUM_TESTS_PER_ITERATION << " others" << std::endl;
    }

    auto test_points = test_point_generator.get_random_point_and_points(NUM_TESTS_PER_ITERATION);
    xyLoc loc1 = test_points.first;

    // Use SimpleDijkstra to find the distance of this point from all others.
    delete[] dijkstra_dists;
    dijkstra_dists = new exact_distance[width * height];
    SimpleDijkstraExactDistance::run_dijkstra(graph.pos(loc1), dijkstra_dists, graph);

    // find a path between loc1 and each other point using corner graphs
    for (xyLoc loc2 : test_points.second)
    {
      std::vector<xyLoc> path;
      GetPath(preprocessing_data, loc1, loc2, path);

      if (path[0].x != loc1.x ||
          path[0].y != loc1.y ||
          path.back().x != loc2.x ||
          path.back().y != loc2.y)
      {
         throw std::runtime_error("path does not match experiment. " + std::to_string(path[0].x) + " vs " + std::to_string(loc1.x) + ", " + std::to_string(path[0].y) + " vs " + std::to_string(loc1.y) + ", " + std::to_string(path.back().x) + " vs " + std::to_string(loc2.x) + ", " + std::to_string(path.back().y) + " vs " + std::to_string(loc2.y));
      }

      // Validate path
      if (!ValidatePath(graph, path))
      {
        print_graph(preprocessing_data, graph, std::vector<map_position>(), path, false, false, false, false, false, false, Dir_NNE, 0);
        throw std::runtime_error("Failed to validate path returned from main. Source: ("  + std::to_string(loc1.x) + ", " + std::to_string(loc1.y) + "). Target: (" + std::to_string(loc2.x) + ", " + std::to_string(loc2.y) + ").");
      }

      // test if this path has the same length as SimpleDijkstra

      exact_distance path_length = ZERO_EXACT_DISTANCE;

      // Find num_straight and num_diagonal for path found using corner graphs
      for (unsigned int i = 1; i < path.size(); i++)
      {
        const xyLoc &prev = path[i-1];
        const xyLoc &curr = path[i];
        if (std::abs(prev.x - curr.x) == 1 && std::abs(prev.y - curr.y) == 1)
          path_length.num_diagonal++;
        else
          path_length.num_straight++;
      }

      // Check that SimpleDijkstra and CornerGraphs return the same length
      exact_distance simple_path_length = dijkstra_dists[graph.pos(loc2)];
      if (path_length != simple_path_length)
      {
        print_graph(preprocessing_data, graph, std::vector<map_position>(), path, false, false, false, false, false, false, Dir_NNE, 0);
        throw std::runtime_error("Test failed from (" + std::to_string(loc1.x) + ", " + std::to_string(loc1.y) + ") to (" + std::to_string(loc2.x) + ", " + std::to_string(loc2.y) + "). Simple had " + std::to_string(simple_path_length.num_straight) + " straight, " + std::to_string(simple_path_length.num_diagonal) + " diagonal. Main had " + std::to_string(path_length.num_straight) + " straight, " + std::to_string(path_length.num_diagonal) + " diagonal.");
      }
    }
  }
  delete[] dijkstra_dists;
}
