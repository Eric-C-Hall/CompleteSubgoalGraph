#include "DirectPath.hpp"

void Running::print_octile_path(xyLoc start, xyLoc end, Printer &printer, const Graph &graph)
{
  std::vector<xyLoc> path;
  Running::compute_octile_path<false>(start, end, path, graph);

  for (xyLoc l : path)
  {
    printer.add_highlight(Highlight(2), l);
  }
  printer.add_highlight(Highlight(3), start);
  printer.add_highlight(Highlight(4), end);
}

