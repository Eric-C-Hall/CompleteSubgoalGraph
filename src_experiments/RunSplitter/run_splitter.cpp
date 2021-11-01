#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <sstream>

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

void expect_next_strings(std::ifstream &f, std::vector<char *> strs)
{
  for (char * s : strs)
    expect_next_string(f, std::string(s));
}

void expect_next_strings(std::ifstream &f, std::string str)
{
  std::stringstream stream(str);
  while (!stream.eof())
  {
    std::string a;
    stream >> a;
    if (stream.fail())
      break;
    //std::cout << a << "\t\t" << str << std::endl;
    expect_next_string(f,a);
  }
}

void expect_next_strings(std::ifstream &f, char const * str)
{
  expect_next_strings(f, std::string(str));
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

  expect_next_strings(f, "Loading preprocessed data Preprocessed data loaded in");
  double pre_load_time;
  f >> pre_load_time;

  std::ofstream output_file(filename + ".runtime");

  while (!f.eof())
  {
    std::string mapname;
    f >> mapname;
    if (f.fail())
      break;

    expect_next_strings(f, "total-time");
    double total_time;
    f >> total_time;

    output_file << total_time << "\n";

    expect_next_strings(f, "max-time-step");
    double max_time_step;
    f >> max_time_step;
    expect_next_strings(f, "time-20-moves");
    double time_20_moves;
    f >> time_20_moves;
    expect_next_strings(f, "total-len");
    double total_len;
    f >> total_len;
    expect_next_strings(f, "subopt");
    double subopt;
    if (total_len == 0)
    {
      // In the special case where the scenario aims to find a path from a point to itself, subopt should be -nan
      expect_next_string(f, "-nan");
      subopt = 1;
    }
    else
    {
      f >> subopt;
    }

    std::string valid_str;
    f >> valid_str;
    bool valid;
    if (valid_str == "valid")
    {
      valid = true;
    }
    else
    {
      expect_string(valid_str, "invalid");
      valid = false;
    }

    if (f.eof())
      break;
  }
  f.close();

  output_file.close();
  return 0;
}
