#include <stdio.h>
#include <stdint.h>
#include <numeric>
#include <algorithm>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "../../src/Preprocessing/Preprocessing.hpp"
#include "../../src/PathCompetition/Entry.h"
#include "../../src/Preprocessing/SmoothedGraph.hpp"

constexpr bool DRAW_SMOOTHED_GRAPH = false;

void LoadMap(const char *fname, std::vector<bool> &map, int &w, int &h);

int main(int argc, char **argv)
{
  std::vector<bool> mapData;
  int width, height;

  if (argc != 2)
  {
    std::cout << "Usage: " << std::string(argv[0]) << " <filename>" << std::endl;
    return -1;
  }

  std::string filename = std::string(argv[1]);
  LoadMap(filename.c_str(), mapData, width, height);
  if (mapData.size() == 0)
  {
    std::cout << "Error opening file: " << filename << std::endl;
    return -1;
  }
  std::cout << "Opened " << filename << std::endl;

  Graph graph;
  graph.load_bits_without_collar(mapData, width, height);

  size_t maps_dir_index = filename.find("maps");
  std::string preprocessing_data_filename = filename.substr(0,maps_dir_index) + std::string(GetName()) + "-" + filename.substr(maps_dir_index);
  PreprocessingData preprocessing_data;
  preprocessing_data.load(preprocessing_data_filename, graph);

  std::string corner_vector_filename = preprocessing_data_filename + ".corner_vector.mem";
  std::string with_relevant_filename = preprocessing_data_filename + ".with_relevant.mem";
  std::string with_next_filename = preprocessing_data_filename + ".with_next.mem";
  std::string ccg_filename = preprocessing_data_filename + ".ccg.mem";
  std::string bb_filename = preprocessing_data_filename + ".bb.mem";

  const CornerVector &corner_vector = preprocessing_data.get_corner_vector();
  const NearbyCornersWithRelevant &with_relevant = preprocessing_data.get_nearby_corners_with_relevant();
  const NearbyCornersWithNext &with_next = preprocessing_data.get_nearby_corners_with_next();
  const CompleteCornerGraph &ccg = preprocessing_data.get_complete_corner_graph();
  const GeometricContainersIncoming &bb = preprocessing_data.get_geometric_containers_incoming();

  std::ofstream corner_vector_file(corner_vector_filename);
  std::ofstream with_relevant_file(with_relevant_filename);
  std::ofstream with_next_file(with_next_filename);
  std::ofstream ccg_file(ccg_filename);
  std::ofstream bb_file(bb_filename);

  corner_vector.save(corner_vector_file);
  with_relevant.save(with_relevant_file, graph, corner_vector);
  with_next.save(with_next_file, graph, corner_vector);
  ccg.save(ccg_file, corner_vector);
  bb.save(bb_file);

  return 0;
}

void LoadMap(const char *fname, std::vector<bool> &map, int &width, int &height)
{
	FILE *f;
	f = fopen(fname, "r");
	if (f)
    {
		fscanf(f, "type octile\nheight %d\nwidth %d\nmap\n", &height, &width);
		map.resize(height*width);
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				char c;
				do {
					fscanf(f, "%c", &c);
				} while (isspace(c));
				map[y*width+x] = (c == '.' || c == 'G' || c == 'S');
				//printf("%c", c);
			}
			//printf("\n");
		}
		fclose(f);
    }
}
