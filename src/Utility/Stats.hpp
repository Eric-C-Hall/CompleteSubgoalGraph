#ifndef CORNERGRAPH_PROJECT__STATS_HPP
#define CORNERGRAPH_PROJECT__STATS_HPP

#include <vector>

// Min, Max, Mean, Median, Mode, First Quartile, Third Quartile
std::tuple<unsigned int, unsigned int, float, unsigned int, unsigned int, unsigned int, unsigned int> get_stats(std::vector<unsigned int> vec);
void print_stats(std::tuple<unsigned int, unsigned int, float, unsigned int, unsigned int, unsigned int, unsigned int> stats);

#endif
