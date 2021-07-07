#include "Preprocessing.hpp"

#include <cassert>
#include <iostream>
#include <fstream>
#include <deque>
#include <tuple>
#include <set>
#include <algorithm>

#include "../Graph/ExactDistance.hpp"
#include "../Graph/Directions.hpp"

#include "../Utility/Timer.h"
#include "../Utility/Stats.hpp"

#include "../Test/Test.hpp"

PreprocessingData::PreprocessingData(const Graph &input_graph) : graph(input_graph)
{
}

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

void PreprocessingData::preprocess()
{
  Timer t;
  double total_time = 0;

  // Compute corners
  start_computation("Computing corners", t);
  corner_vector.preprocess(graph);
  end_computation("Corners computed", t, total_time);
  
  graph.print();

  // Find nearby corners
  start_computation("Finding nearby corners", t);
  nearby_corners.preprocess(graph, corner_vector);
  end_computation("Nearby corners found", t, total_time);
  
  // Find complete corner graph
  start_computation("Finding complete corner graph", t);
  complete_corner_graph.preprocess(graph, corner_vector, nearby_corners);
  end_computation("Complete corner graph found",t,total_time);
  
  std::cout << "Preprocessing complete" << std::endl;
  std::cout << "Total preprocessing time: " << total_time << std::endl;
}

void PreprocessingData::_save(std::ostream & stream) const
{
  corner_vector.save(stream);
  nearby_corners.save(stream, graph, corner_vector);
  complete_corner_graph.save(stream, graph, corner_vector);
}

void PreprocessingData::save(const std::string &filename) const
{
  Timer t; double total_time;
  start_computation("Saving preprocessed data", t);
  std::ofstream file(filename, std::ifstream::out | std::ifstream::binary);
  if (file.fail())
  {
    std::cerr << "Attempt to save \"" << filename << "\"" << std::endl;
    throw std::runtime_error("Error saving preprocessed data file.");
  }
  _save(file);
  file.close();
  end_computation("Preprocessed data saved", t, total_time);
}

void PreprocessingData::_load(std::istream &stream)
{
  corner_vector.load(stream);
  nearby_corners.load(stream, graph, corner_vector);
  complete_corner_graph.load(stream, graph, corner_vector);
}

void PreprocessingData::load(const std::string &filename)
{
  Timer t; double total_time;
  start_computation("Loading preprocessed data", t);
  std::ifstream file(filename, std::ifstream::in | std::ifstream::binary);
  if (file.fail())
  {
    std::cerr << "Attempt to open \"" << filename << "\"" << std::endl;
    throw std::runtime_error("Error opening preprocessed data file.");
  }
  _load(file);
  end_computation("Preprocessed data loaded", t, total_time);
}
