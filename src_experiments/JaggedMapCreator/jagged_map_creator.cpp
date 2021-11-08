#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <cstdlib>

constexpr int MAP_SIZE = 50;

void output_header(std::ofstream &stream)
{
  stream << "type octile\nheight " << MAP_SIZE << "\nwidth " << MAP_SIZE << "\nmap\n";
}

void init_obstacles(std::vector<std::vector<bool>> &is_obstacle)
{
  is_obstacle.resize(MAP_SIZE);
  for (int y = 0; y < MAP_SIZE; y++)
  {
    is_obstacle[y].resize(MAP_SIZE, false);
  }
}

void set_row_to_obstacles(std::vector<std::vector<bool>> &is_obstacle, int which_row)
{
  is_obstacle[which_row].clear();
  is_obstacle[which_row].resize(MAP_SIZE, true);
}

void set_column_to_obstacles(std::vector<std::vector<bool>> &is_obstacle, int which_column)
{
  for (auto &row : is_obstacle)
  {
    row[which_column] = true;
  }
}

void set_half_row_to_obstacles(std::vector<std::vector<bool>> &is_obstacle, int which_row)
{
  for (int i = 0; i < MAP_SIZE/2; i++)
  {
    is_obstacle[which_row][i] = true;
  }
}

void set_row_to_jagged(std::vector<std::vector<bool>> &is_obstacle, int which_row)
{
  for (int i = 0; i < MAP_SIZE - 1; i += 4)
  {
    is_obstacle[which_row][i] = true;
    is_obstacle[which_row][i+1] = true;
  }
}

void set_half_row_to_jagged(std::vector<std::vector<bool>> &is_obstacle, int which_row)
{
  int i;
  for (i = 0; i < MAP_SIZE/2 - 1; i += 4)
  {
    is_obstacle[which_row][i] = true;
    is_obstacle[which_row][i+1] = true;
  }
  if (i == MAP_SIZE/2 - 1)
    is_obstacle[which_row][i] = true;
}

void set_column_to_jagged(std::vector<std::vector<bool>> &is_obstacle, int which_column)
{
  for (int i = 0; i < MAP_SIZE - 1; i += 4)
  {
    is_obstacle[i][which_column] = true;
    is_obstacle[i+1][which_column] = true;
  }
}

void set_back_half_column_to_jagged(std::vector<std::vector<bool>> &is_obstacle, int which_column)
{
  for (int i = MAP_SIZE/2; i < MAP_SIZE - 1; i += 4)
  {
    is_obstacle[i][which_column] = true;
    is_obstacle[i+1][which_column] = true;
  }
}

void output_obstacles(std::ofstream &output, const std::vector<std::vector<bool>> is_obstacle)
{
  bool is_first_time = true;
  for (const auto &row : is_obstacle)
  {
    if (!is_first_time)
      output << "\n";
    else
      is_first_time = false;
    for (const bool &value : row)
    {
      output << (value ? "@" : ".");
    }
  }
}

int main(int argc, char **argv)
{
  // Set up smooth map obstacles
  std::vector<std::vector<bool>> is_obstacle;
  init_obstacles(is_obstacle);

  set_row_to_obstacles(is_obstacle, 0);
  set_row_to_obstacles(is_obstacle, MAP_SIZE-1);
  set_column_to_obstacles(is_obstacle, 0);
  set_column_to_obstacles(is_obstacle, MAP_SIZE-1);

  for (int i = MAP_SIZE/2; i < MAP_SIZE; i++)
  {
    set_half_row_to_obstacles(is_obstacle, i);
  }


  // Output smooth map
  std::ofstream smooth_map_output("smoothmap.map");
  output_header(smooth_map_output);
  output_obstacles(smooth_map_output, is_obstacle);
  smooth_map_output.close();

  // Modify for jagged map
  set_row_to_jagged(is_obstacle, 1);
  set_row_to_jagged(is_obstacle, MAP_SIZE-2);
  set_column_to_jagged(is_obstacle, 1);
  set_column_to_jagged(is_obstacle, MAP_SIZE-2);
  set_half_row_to_jagged(is_obstacle, (MAP_SIZE/2)-1);
  set_back_half_column_to_jagged(is_obstacle, MAP_SIZE/2);

  // Output jagged map
  std::ofstream jagged_map_output("jaggedmap.map");
  output_header(jagged_map_output);
  output_obstacles(jagged_map_output, is_obstacle);
  jagged_map_output.close();

  return 0;
}
