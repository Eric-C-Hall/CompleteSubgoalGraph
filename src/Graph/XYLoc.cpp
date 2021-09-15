#include "XYLoc.hpp"

bool xyLocLessThan::operator()(const xyLoc &a, const xyLoc &b)
{
  if (a.x != b.x)
  {
    return a.x < b.x;
  }
  return a.y < b.y;
}


std::ostream& operator<< (std::ostream& stream, const xyLoc& loc)
{
  stream << "(" << loc.x << "," << loc.y << ")";
  return stream;
}
