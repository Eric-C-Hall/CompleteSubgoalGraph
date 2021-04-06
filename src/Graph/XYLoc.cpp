#include "XYLoc.hpp"

xyLoc::xyLoc()
{
}

xyLoc::xyLoc(const xyLoc &other)
{
  x = other.x;
  y = other.y;
}

xyLoc::xyLoc(int16_t input_x, int16_t input_y)
{
  x = input_x;
  y = input_y;
}

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
