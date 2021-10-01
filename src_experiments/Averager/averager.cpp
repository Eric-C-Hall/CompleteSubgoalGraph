#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <cstdlib>

std::string filename;

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cout << "Usage: " << std::string(argv[0]) << " <filename>" << std::endl;
    return -1;
  }

  filename = std::string(argv[1]);

  std::ifstream f(filename);

  double total = 0;
  int num_doubles = 0;
  // Read file for total times
  while (!f.eof())
  {
    double d;
    f >> d;

    if (f.fail())
      break;

    total += d;
    num_doubles++;
  }
  std::cout << (total / num_doubles) << std::endl;
  
  f.close();
}
