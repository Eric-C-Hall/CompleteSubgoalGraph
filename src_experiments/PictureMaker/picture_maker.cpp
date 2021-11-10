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

  if (argc != 6)
  {
    std::cout << "Usage: " << std::string(argv[0]) << " <filename> <xMin> <yMin> <xMax> <yMax>" << std::endl;
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
  std::string output_filename = "pics/" + filename.substr(5, filename.length() - 9) + "_picture.tex";
  std::cout << "Outputting to " << output_filename << std::endl;
  std::ofstream output(output_filename.c_str());

  std::string xMinStr, yMinStr, xMaxStr, yMaxStr;
  xMinStr = std::string(argv[2]);
  yMinStr = std::string(argv[3]);
  xMaxStr = std::string(argv[4]);
  yMaxStr = std::string(argv[5]);
  
  int xMin, yMin, xMax, yMax;
  xMin = std::stoi(xMinStr);
  yMin = std::stoi(yMinStr);
  xMax = std::stoi(xMaxStr);
  yMax = std::stoi(yMaxStr);

  if (xMin > xMax)
    std::swap(xMin, xMax);

  if (yMin > yMax)
    std::swap(yMin, yMax);

  int viewWidth, viewHeight;
  viewWidth = (xMax - xMin) + 1;
  viewHeight = (yMax - yMin) + 1;

  if (viewWidth != viewHeight)
  {
    std::cout << "View window must be square"              << std::endl;
    std::cout << "i.e. xMax - xMin must equal yMax - yMin" << std::endl;
    return -1;
  }

  double stepSize = (10 / (double)viewWidth);

  output << "\\begin{figure}"        << "\n";
  output << "  \\centering"          << "\n";
  output << "  \\begin{tikzpicture}" << "\n";

  output << "  % xMin: " << xMin << ", yMin: " << yMin << ", xMax: " << xMax << ", yMax: " << yMax << "\n";
  output << "  % filename: " << filename << "\n";

  output << "\n";

  // Draw obstacles
  for (int y = yMin; y <= yMax; y++)
  {
    int yDelta = y - yMin;

    for (int xRunBegin = xMin; xRunBegin <= xMax; xRunBegin++)
    {
      int xRunEnd;
      for (xRunEnd = xRunBegin; xRunEnd <= xMax && !mapData[y * width + xRunEnd]; xRunEnd++)
      {
      }
      if (xRunBegin != xRunEnd)
      {
        int xRunBeginDelta = xRunBegin - xMin;
        int xRunEndDelta = xRunEnd - xMin;
        output << "  \\draw[fill, color=black] (" << xRunBeginDelta * stepSize << ", " << yDelta * stepSize << ") rectangle (" << xRunEndDelta * stepSize << ", " << (yDelta + 1) * stepSize << ");\n";
      }
      xRunBegin = xRunEnd;
    }
  }
  output << "\n";

  Graph graph;
  graph.load_bits_without_collar(mapData, width, height);
  PreprocessingData preprocessing_data;
  preprocessing_data.load(std::string(GetName()) + "-" + filename, graph);
  const CornerVector &corner_vector = preprocessing_data.get_corner_vector();
  SmoothedGraph smoothed_graph;
  smoothed_graph.preprocess(graph, corner_vector);

  if (DRAW_SMOOTHED_GRAPH)
  {
    // Draw smoothed graph
    for (int y = yMin; y <= yMax; y++)
    {
      // Draw smoothed graph obstacles
      int yDelta = y - yMin;
      for (int xRunBegin = xMin; xRunBegin <= xMax; xRunBegin++)
      {
        int xRunEnd;
        map_position xRunEndPos;

        for (xRunEnd = xRunBegin;
             xRunEndPos = graph.pos(xRunEnd, y), xRunEnd <= xMax && !graph.is_obstacle(xRunEndPos) && smoothed_graph.get_is_blocked(xRunEndPos);
             xRunEnd++)
        {
        }
        if (xRunBegin != xRunEnd)
        {
          int xRunBeginDelta = xRunBegin - xMin;
          int xRunEndDelta = xRunEnd - xMin;
          output << "  \\draw[fill, color=yellow] (" << xRunBeginDelta * stepSize << ", " << yDelta * stepSize << ") rectangle (" << xRunEndDelta * stepSize << ", " << (yDelta + 1) * stepSize << ");\n";
        }
        xRunBegin = xRunEnd;
      }
    }
  }

  // Draw corners
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    map_position corner_pos = corner_vector.get_corner(i);
    xyLoc corner_loc = graph.loc(corner_pos);
    if (xMin <= corner_loc.x && corner_loc.x <= xMax && yMin <= corner_loc.y && corner_loc.y <= yMax)
    {
      int xDelta = corner_loc.x - xMin;
      int yDelta = corner_loc.y - yMin;

      std::string color_string;
      if (DRAW_SMOOTHED_GRAPH && smoothed_graph.get_is_blocked(corner_pos))
      {
        color_string = "orange";
      }
      else if (DRAW_SMOOTHED_GRAPH && !smoothed_graph.get_is_corner(corner_pos))
      {
        color_string = "red";
      }
      else
      {
        color_string = "cyan";
      }

      output << "  \\draw[fill, color=" << color_string << "] (" << xDelta * stepSize << ", " << yDelta * stepSize << ") rectangle (" << (xDelta + 1) * stepSize << ", " << (yDelta + 1) * stepSize << ");\n";
    }
  }
  output << "\n";

  output << "\n";
  output << "  \\draw[step=" << stepSize << ", black, thin] (0,0) grid (10,10);" << "\n";
  output << "  \\end{tikzpicture}" << "\n";
  output << "  \\caption{No caption}" << "\n";
  output << "  \\label{fig:unlabelled}" << "\n";
  output << "\\end{figure}" << std::endl;

  output.close();

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
