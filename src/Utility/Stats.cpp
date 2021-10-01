#include "Stats.hpp"

#include <algorithm>
#include <map>
#include <iostream>

// Min, Max, Mean, Median, Mode, First Quartile, Third Quartile
std::tuple<int, int, float, int, int, int, int> Stats::get_stats(std::vector<int> vec)
{
  // Sorting the vector makes it much easier to find this data
  std::sort(vec.begin(), vec.end());

  // Min, max
  int min = vec[0];
  int max = vec[vec.size()-1];

  // Mean
  int total = 0;
  for (int i : vec)
  {
    total += i;
  }
  float mean = ((float)total)/((float)vec.size());

  // Median
  int median = vec[vec.size()/2];

  // Mode
  std::map<int, int> freq_of_value;
  for (int i : vec)
  {
    if (freq_of_value.find(i) == freq_of_value.end())
      freq_of_value[i] = 0;
    freq_of_value[i]++;
  }
  int highest_freq = 0;
  int value_with_highest_freq = 0;
  for (auto key_value : freq_of_value)
  {
    if (key_value.second >= highest_freq)
    {
      highest_freq = key_value.second;
      value_with_highest_freq = key_value.first;
    }
  }
  int mode  = value_with_highest_freq;

  // First Quartile
  int qt1 = vec[vec.size()/4];

  // Third Quartile
  int qt3 = vec[(vec.size()*3)/4];

  return std::tuple<int, int, float, int, int, int, int>
                   (min, max, mean, median, mode, qt1, qt3);
  
}

void Stats::print_stats(std::tuple<int, int, float, int, int, int, int> stats)
{
  std::cout << "Min:    " << std::get<0>(stats) << std::endl;
  std::cout << "Qt1:    " << std::get<5>(stats) << std::endl;
  std::cout << "Median: " << std::get<3>(stats) << std::endl;
  std::cout << "Qt2:    " << std::get<6>(stats) << std::endl;
  std::cout << "Max:    " << std::get<1>(stats) << std::endl;
  std::cout << "Mean:   " << std::get<2>(stats) << std::endl;
  std::cout << "Mode:   " << std::get<4>(stats) << std::endl;
}

std::vector<int> Stats::to_histogram(std::vector<int> vec)
{
  std::sort(vec.begin(), vec.end());

  std::vector<int> return_value(vec.back()+1, 0);

  auto iter = vec.begin();
  while (iter != vec.end())
  {
    int curr_value = *iter;
    int num_values = 0;
    while (iter != vec.end() && *iter == curr_value)
    {
      num_values++;
      iter++;
    }
    return_value[curr_value] = num_values;
  }

  return return_value;
}

void Stats::print_histogram(const std::vector<int> &hist, bool newline_and_flush)
{
  for (size_t n = 0; n < hist.size(); n++)
  {
    if (n != 0)
      std::cout << " ";
    std::cout << hist[n];
  }
  if (newline_and_flush)
    std::cout << std::endl;
}
