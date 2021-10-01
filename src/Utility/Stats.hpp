#ifndef CORNERGRAPH_PROJECT__STATS_HPP
#define CORNERGRAPH_PROJECT__STATS_HPP

#include <vector>
#include <tuple>

// Min, Max, Mean, Median, Mode, First Quartile, Third Quartile
namespace Stats
{
  std::tuple<int, int, float, int, int, int, int> get_stats(std::vector<int> vec);
  void print_stats(std::tuple<int, int, float, int, int, int, int> stats);

  std::vector<int> to_histogram(std::vector<int> vec);
  void print_histogram(const std::vector<int> &hist, bool newline_and_flush = true);
}

#endif
