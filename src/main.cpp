#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include <string>
#include <numeric>
#include <random>
#include <algorithm>

#include "Graph/Graph.hpp"
#include "Preprocessing/Preprocessing.hpp"
#include "Run/RunScenario/RunScenario.hpp"
#include "Test/Test.hpp"
#include "Visualise/VisualisePreprocessed.hpp"
#include "Time/Time.hpp"

const std::string ALGORITHM_NAME("cornergraph");

void print_usage(const std::string &program_name)
{
  printf("Invalid Arguments\nUsage %s <flag> <map> <scenario>\n", program_name.c_str());
  printf("Flags:\n");
  printf("\t-pre : Preprocess map\n");
  printf("\t-run : Run scenario without preprocessing\n");
  printf("\t-test : Test preprocessed map\n");
  printf("\t-time : Time brigitte on preprocessed map\n");
}

// ------------
// Entry point
// ------------
int main(int argc, char **argv)
{
  std::string program_name;
  std::string operating_mode;
  std::string map_filename;
  std::string scen_filename;
  std::string processed_filename;

  bool pre = false;
  bool run = false;
  bool vis = false;
  bool test = false;
  bool time = false;
  bool load = true;

  // Ensure number of arguments is correct
  if (argc != 4)
  {
    print_usage(std::string(argv[0]));
    exit(0);
  }

  // Argument 0: name of program (technically this isn't quite true, if you run
  //             with ./cornergraph, this will be ./cornergraph rather than cornergraph)
  // Argument 1: operating mode
  // Argument 2: map filename
  // Argument 3: scenario filename
  program_name = std::string(argv[0]);
  operating_mode = std::string(argv[1]);
  map_filename = std::string(argv[2]);
  scen_filename = std::string(argv[3]);

  // Determine filename for the preprocessed map based on the map filename
  processed_filename = map_filename + "." + ALGORITHM_NAME;

  Graph graph;
  graph.load_map(map_filename.c_str());
  //graph.debug_cut_sides(0,0,0,0);

  // Determine which operating mode we should run in
  if (operating_mode.compare("-pre") == 0)
    pre = true, load = false;
  else if (operating_mode.compare("-run") == 0)
    run = true;
  else if (operating_mode.compare("-vis") == 0)
    vis = true;
  else if (operating_mode.compare("-test") == 0)
    test = true;
  else if (operating_mode.compare("-time") == 0)
    time = true;
  else {
    print_usage(program_name);
    exit(0);
  }

  PreprocessingData preprocessing_data(graph);

  // If desired, preprocess the map
  if (pre)
  {
    preprocessing_data.preprocess();
    preprocessing_data.save(processed_filename.c_str());
  }
  
  // If necessary, load the preprocessed map
  if (load)
    preprocessing_data.load(processed_filename.c_str());

  // If desired, visualise the map
  if (vis)
    Visualise(preprocessing_data);

  // If desired, test whether the results on the preprocessed map are what we would expect
  if (test)
    Test(preprocessing_data);

  // If desired, time Brigitte
  if (time)
    Time(preprocessing_data, 10000);

  // If desired, test how fast the algorithm performs
  if (run)
  {
    RunScenario(scen_filename, preprocessing_data);
  }

  return 0;
}
