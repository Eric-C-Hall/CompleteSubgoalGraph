#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <cstdlib>

std::string filename;

void expect_string(std::string s, std::string expected)
{
  if (s != expected)
  {
    std::cout << "Expected string \"" << expected << "\", instead got \"" << s << "\", filename: " << filename << std::endl;
    exit(EXIT_FAILURE);
  }
}

void expect_next_string(std::ifstream &f, std::string expected)
{
  std::string next;
  f >> next;
  expect_string(next, expected);
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cout << "usage: " << argv[0] << " " << "filename" << std::endl;
    return -1;
  }

  filename = std::string(argv[1]);
  //std::cout << "Filename: " << filename << std::endl;
  std::ifstream f(filename);

  for (const char * other : {"Loading", "preprocessed", "data", "Preprocessed", "data", "loaded", "in"})
  {
    expect_next_string(f,std::string(other));
  }

  double preprocessed_data_load_time;
  f >> preprocessed_data_load_time;

  std::vector<std::string> scenarios;
  std::vector<double> total_times;
  std::vector<double> max_time_steps;
  std::vector<double> time_20_moveses;
  std::vector<double> total_lengths;
  std::vector<double> subopts;
  std::vector<bool> is_valids;
  
  // Read file for total times
  while (!f.eof())
  {
    std::string scenario_name;
    f >> scenario_name;
    if (f.eof())
      break;
    scenarios.push_back(scenario_name);

    expect_next_string(f, std::string("total-time"));

    double total_time;
    f >> total_time;
    total_times.push_back(total_time);

    expect_next_string(f, std::string("max-time-step"));

    double max_time_step;
    f >> max_time_step;
    max_time_steps.push_back(max_time_step);

    expect_next_string(f, std::string("time-20-moves"));

    double time_20_moves;
    f >> time_20_moves;
    time_20_moveses.push_back(time_20_moves);

    expect_next_string(f, std::string("total-len"));

    double total_length;
    f >> total_length;
    total_lengths.push_back(total_length);

    expect_next_string(f, std::string("subopt"));

    double subopt;
    if (total_length == 0)
    {
      // In the special case where the scenario aims to find a path from a point to itself, subopt should be -nan
      expect_next_string(f, "-nan");
      subopts.push_back(1);
    }
    else
    {
      f >> subopt;
      subopts.push_back(subopt);
    }

    std::string valid_str;
    f >> valid_str;
    if (valid_str == "valid")
    {
      is_valids.push_back(true);
    }
    else
    {
      expect_string(valid_str, "invalid");
      is_valids.push_back(false);
    }
  }
  f.close();

  for (bool b : is_valids)
  {
    if (!b)
    {
      std::cout << "Some path is not valid" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  for (double d : subopts)
  {
    if (d < 0.999990 || 1.000001 < d)
    {
      std::cout << "Some path is excessively non-optimal" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  //std::cout << "All paths are valid and sufficiently optimal" << std::endl;

  return 0;
}
