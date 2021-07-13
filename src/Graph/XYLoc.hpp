#ifndef CORNERGRAPH_PROJECT__XYLOC_HPP
#define CORNERGRAPH_PROJECT__XYLOC_HPP

#include <iostream>

struct xyLoc {
  xyLoc();
  xyLoc(const xyLoc &other);
  xyLoc(int16_t input_x, int16_t input_y);
  int16_t x;
  int16_t y;

  xyLoc operator+(const xyLoc &other) const
  {
    return xyLoc(x + other.x, y + other.y);
  }

  xyLoc operator-(const xyLoc &other) const
  {
    return xyLoc(x - other.x, y - other.y);
  }

  bool operator==(const xyLoc &other) const
  {
    return x == other.x && y == other.y;
  }

  bool operator!=(const xyLoc &other) const
  {
    return x != other.x || y != other.y;
  }
};

struct xyLocLessThan
{
  bool operator()(const xyLoc &a, const xyLoc &b); 
};

std::ostream& operator<< (std::ostream& stream, const xyLoc& loc);

inline xyLoc up(xyLoc l) {return xyLoc(l.x, l.y + 1);}
inline xyLoc down(xyLoc l) {return xyLoc(l.x, l.y - 1);}
inline xyLoc left(xyLoc l) {return xyLoc(l.x - 1, l.y);}
inline xyLoc right(xyLoc l) {return xyLoc(l.x + 1, l.y);}

#endif
