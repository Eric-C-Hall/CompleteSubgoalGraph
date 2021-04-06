#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include <string>
#include <numeric>
#include <random>
#include <algorithm>

#include "Graph/Graph.hpp"
#include "Preprocess/Preprocess.hpp"
#include "Run/RunScenario/RunScenario.hpp"

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
  bool test = false;
  bool time = false;
  bool load = false;

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

  // Determine which operating mode we should run in
  if (operating_mode.compare("-pre") == 0)
    pre = true;
  else if (operating_mode.compare("-run") == 0)
    load = run = true;
  else if (operating_mode.compare("-test") == 0)
    load = test = true;
  else if (operating_mode.compare("-time") == 0)
    load = time = true;
  else {
    print_usage(program_name);
    exit(0);
  }

  PreprocessingData preprocessing_data(graph);

  // If desired, preprocess the map
  // (see Preprocess.cpp)
  if (pre)
  {
    preprocessing_data.preprocess();
    preprocessing_data.save(processed_filename.c_str());
  }
  
  // If necessary, load the preprocessed map
  // (see Entry.cpp)
  if (load)
    preprocessing_data.load(processed_filename.c_str());

  // If desired, test whether the results on the preprocessed map are what we would expect
  // (see TestBrigitte.cpp)
  //if (test)
  //  Test(reference, width, height, mapData);

  // If desired, time Brigitte
  // (see TimeBrigitte.cpp)
  //if (time)
  //  TimeBrigitte(reference, width, height, mapData, 10000);

  // If desired, test how fast the algorithm performs
  // (see RunScenario.cpp)
  if (run)
  {
    RunScenario(scen_filename, preprocessing_data);
  }

  return 0;
}
