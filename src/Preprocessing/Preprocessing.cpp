#include "Preprocessing.hpp"

#include <cassert>
#include <iostream>
#include <fstream>
#include <deque>
#include <tuple>
#include <set>
#include <algorithm>

#include "RelevantPoints.hpp"
#include "CornerVectorSplittable.hpp"
#include "DegreeTwoCorners.hpp"
#include "SmoothedGraph.hpp"

#include "../Graph/ExactDistance.hpp"
#include "../Graph/Directions.hpp"

#include "../PathCompetition/Timer.h"
#include "../Utility/Stats.hpp"

#include "../Test/Test.hpp"

void start_computation(std::string message, Timer &t)
{
  std::cout << message << std::endl;
  t.StartTimer();
}

void end_computation(std::string message, Timer &t, double &total_time)
{
  t.EndTimer();
  std::cout << message << " in " << t.GetElapsedTime() << std::endl;
  total_time += t.GetElapsedTime();
  std::cout << std::endl;
}

void PreprocessingData::preprocess(const Graph &graph)
{
  Timer t;
  double total_time = 0;

  // Compute corners
  start_computation("Computing corners", t);
  corner_vector.preprocess(graph);
  end_computation("Corners computed", t, total_time);

  // Compute smoothed graph
  SmoothedGraph smoothed_graph;
  start_computation("Computing smoothed graph", t);
  smoothed_graph.preprocess(graph, corner_vector);
  end_computation("Smoothed graph computed", t, total_time);

  // Find nearby corners
  NearbyCorners nearby_corners;
  start_computation("Finding nearby corners", t);
  nearby_corners.preprocess(graph, corner_vector);
  end_computation("Nearby corners found", t, total_time);

  // Find complete corner graph
  start_computation("Finding complete corner graph", t);
  complete_corner_graph.preprocess(graph, corner_vector, nearby_corners);
  end_computation("Complete corner graph found", t, total_time);

  // Find degree two corners
  DegreeTwoCorners degree_two_corners;
  start_computation("Finding degree two corners", t);
  degree_two_corners.preprocess(graph, corner_vector, nearby_corners, complete_corner_graph);
  end_computation("Degree two corners found", t, total_time);

  // Find relevant points, corners, etc
  RelevantPoints relevant_points;
  start_computation("Finding relevant points", t);
  relevant_points.preprocess(graph, corner_vector, nearby_corners);
  end_computation("Relevant points found", t, total_time);

  // Find nearby corners with relevant
  start_computation("Finding nearby corners with relevant", t);
  nearby_corners_with_relevant.preprocess(graph, corner_vector, nearby_corners, relevant_points);
  end_computation("Nearby corners with relevant found", t, total_time);

  // Find nearby corners with next
  start_computation("Finding nearby corners with next", t);
  nearby_corners_with_next.preprocess(graph, corner_vector, nearby_corners, relevant_points);
  end_computation("Nearby corners with next found", t, total_time);

  // Find corners with no next corners in important directions
  CornerVectorSplittable corner_vector_splittable;
  start_computation("Finding splittable corners", t);
  corner_vector_splittable.preprocess(graph, corner_vector, relevant_points);
  end_computation("Splittable corners found", t, total_time);

  // Geometric containers outgoing
  GeometricContainersOutgoing geometric_containers_outgoing;
  start_computation("Finding geometric containers", t);
  geometric_containers_outgoing.preprocess(graph, corner_vector, complete_corner_graph, relevant_points);
  end_computation("Geometric containers found", t, total_time);
  
  // Convert outgoing geometric containers to incoming geometric containers
  start_computation("Converting geometric containers", t);
  geometric_containers_incoming.convert_from(geometric_containers_outgoing, graph, corner_vector, relevant_points);
  end_computation("Geometric containers converted", t, total_time);

  // Push first corners
  start_computation("Pushing first corners", t);
  complete_corner_graph.push_first_corners(graph, corner_vector);
  end_computation("First corners pushed", t, total_time);

  std::cout << "Preprocessing complete" << std::endl;
  std::cout << "Total preprocessing time: " << total_time << std::endl;

  // Print stats
  relevant_points.print_num_relevant_corner_stats(corner_vector);
}

// Note: does not remove graph's collar
void PreprocessingData::remove_collar(const Graph &graph)
{
  corner_vector.remove_collar(graph);
  nearby_corners_with_relevant.remove_collar(graph);
  nearby_corners_with_next.remove_collar(graph);
  complete_corner_graph.remove_collar(graph);
  geometric_containers_incoming.remove_collar(graph);
}

void PreprocessingData::_save(std::ostream & stream, const Graph &graph) const
{
  corner_vector.save(stream);
  nearby_corners_with_relevant.save(stream, graph, corner_vector);
  nearby_corners_with_next.save(stream, graph, corner_vector);
  complete_corner_graph.save(stream, corner_vector);
  geometric_containers_incoming.save(stream);
}

void PreprocessingData::save(const std::string &filename, const Graph &graph) const
{
  Timer t; double total_time;
  start_computation("Saving preprocessed data", t);
  std::ofstream file(filename, std::ifstream::out | std::ifstream::binary);
  if (file.fail())
  {
    std::cerr << "Attempt to save \"" << filename << "\"" << std::endl;
    throw std::runtime_error("Error saving preprocessed data file.");
  }
  _save(file, graph);
  file.close();
  end_computation("Preprocessed data saved", t, total_time);
}

void PreprocessingData::_load(std::istream &stream, const Graph &graph)
{
  corner_vector.load(stream);
  nearby_corners_with_relevant.load(stream, graph, corner_vector);
  nearby_corners_with_next.load(stream, graph, corner_vector);
  complete_corner_graph.load(stream, corner_vector);
  geometric_containers_incoming.load(stream, corner_vector);
}

void PreprocessingData::load(const std::string &filename, const Graph &graph)
{
  Timer t; double total_time;
  start_computation("Loading preprocessed data", t);
  std::ifstream file(filename, std::ifstream::in | std::ifstream::binary);
  if (file.fail())
  {
    std::cerr << "Attempt to open \"" << filename << "\"" << std::endl;
    throw std::runtime_error("Error opening preprocessed data file.");
  }
  _load(file, graph);
  end_computation("Preprocessed data loaded", t, total_time);
}
