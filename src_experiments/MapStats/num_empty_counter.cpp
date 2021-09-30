#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include <numeric>
#include <algorithm>
#include <vector>

void LoadMap(const char *fname, std::vector<bool> &map, int &w, int &h);

int main(int argc, char **argv)
{
  std::vector<bool> mapData;
  int width, height;
	
  LoadMap(argv[1], mapData, width, height);

  int num_empty = 0;
  for (bool b : mapData)
    if (b)
      num_empty++;

  std::cout << num_empty << std::endl;

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
