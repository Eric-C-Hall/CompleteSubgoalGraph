#ifndef CORNERGRAPH_PROJECT__XYLOC_HPP
#define CORNERGRAPH_PROJECT__XYLOC_HPP

#include <iostream>

#include "../PathCompetition/Entry.h"

inline xyLoc make_xyLoc(int16_t x, int16_t y)
{
  xyLoc return_value;
  return_value.x = x;
  return_value.y = y;
  return return_value;
}

inline xyLoc operator+(const xyLoc &a, const xyLoc &b)
{
  return make_xyLoc(a.x + b.x, a.y + b.y);
}

inline xyLoc operator-(const xyLoc &a, const xyLoc&b)
{
  return make_xyLoc(a.x - b.x, a.y - b.y);
}

inline bool operator==(const xyLoc &a, const xyLoc &b)
{
  return a.x == b.x && a.y == b.y;
}

inline bool operator!=(const xyLoc &a, const xyLoc &b)
{
  return a.x != b.x || a.y != b.y;
}


struct xyLocLessThan
{
  bool operator()(const xyLoc &a, const xyLoc &b); 
};

std::ostream& operator<< (std::ostream& stream, const xyLoc& loc);

inline xyLoc up(xyLoc l) {return make_xyLoc(l.x, l.y + 1);}
inline xyLoc down(xyLoc l) {return make_xyLoc(l.x, l.y - 1);}
inline xyLoc left(xyLoc l) {return make_xyLoc(l.x - 1, l.y);}
inline xyLoc right(xyLoc l) {return make_xyLoc(l.x + 1, l.y);}

#endif
