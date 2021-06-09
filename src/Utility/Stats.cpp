#include "Stats.hpp"

#include <algorithm>
#include <map>
#include <iostream>

// Min, Max, Mean, Median, Mode, First Quartile, Third Quartile
std::tuple<unsigned int, unsigned int, float, unsigned int, unsigned int, unsigned int, unsigned int> get_stats(std::vector<unsigned int> vec)
{
  // Sorting the vector makes it much easier to find this data
  std::sort(vec.begin(), vec.end());

  // Min, max
  unsigned int min = vec[0];
  unsigned int max = vec[vec.size()-1];

  // Mean
  unsigned int total = 0;
  for (unsigned int i : vec)
  {
    total += i;
  }
  float mean = ((float)total)/((float)vec.size());

  // Median
  unsigned int median = vec[vec.size()/2];

  // Mode
  std::map<unsigned int, unsigned int> freq_of_value;
  for (unsigned int i : vec)
  {
    freq_of_value[i]++;
  }
  unsigned int highest_freq = 0;
  unsigned int value_with_highest_freq = 0;
  for (auto key_value : freq_of_value)
  {
    if (key_value.second >= highest_freq)
    {
      highest_freq = key_value.second;
      value_with_highest_freq = key_value.first;
    }
  }
  unsigned int mode  = value_with_highest_freq;

  // First Quartile
  unsigned int qt1 = vec[vec.size()/4];

  // Third Quartile
  unsigned int qt3 = vec[(vec.size()*3)/4];

  return std::tuple<unsigned int, unsigned int, float,
                    unsigned int, unsigned int, unsigned int,
                    unsigned int>(min, max, mean, median, mode, qt1, qt3);
  
}

void print_stats(std::tuple<unsigned int, unsigned int, float, unsigned int, unsigned int, unsigned int, unsigned int> stats)
{
  std::cout << "Min:    " << std::get<0>(stats) << std::endl;
  std::cout << "Qt1:    " << std::get<5>(stats) << std::endl;
  std::cout << "Median: " << std::get<3>(stats) << std::endl;
  std::cout << "Qt2:    " << std::get<6>(stats) << std::endl;
  std::cout << "Max:    " << std::get<1>(stats) << std::endl;
  std::cout << "Mean:   " << std::get<2>(stats) << std::endl;
  std::cout << "Mode:   " << std::get<4>(stats) << std::endl;
}
