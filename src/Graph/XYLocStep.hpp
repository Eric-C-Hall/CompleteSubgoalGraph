#ifndef CORNERGRAPH_PROJECT__XYLOC_STEP_HPP
#define CORNERGRAPH_PROJECT__XYLOC_STEP_HPP

#include "XYLoc.hpp"
#include "Directions.hpp"

inline xyLoc step_in_direction(xyLoc l, Direction dir);
inline xyLoc step_in_direction(xyLoc l, DivDirection divdirection);

// ----------------
// Implementations
// ----------------

inline xyLoc step_in_direction(xyLoc l, Direction dir)
{
  switch (dir) {
  case Dir_N:
  case Dir_NE:
  case Dir_NW:
    l = up(l);
    break;
  case Dir_S:
  case Dir_SE:
  case Dir_SW:
    l = down(l);
    break;
  default:
    break;
  }
  
  switch (dir) {
  case Dir_E:
  case Dir_NE:
  case Dir_SE:
    l = right(l);
    break;
  case Dir_W:
  case Dir_NW:
  case Dir_SW:
    l = left(l);
    break;
  default:
    break;
  }

  return l;
}

inline xyLoc step_in_direction(xyLoc l, DivDirection divdirection)
{
  switch (divdirection)
  {
    case DivDir_NNE:
    case DivDir_NNW:
      l = up(l);
      // No break here is deliberate
    case DivDir_N:
    case DivDir_NE:
    case DivDir_NW:
    case DivDir_ENE:
    case DivDir_WNW:
      l = up(l);
      break;
    case DivDir_SSE:
    case DivDir_SSW:
      l = down(l);
      // No break here is deliberate
    case DivDir_S:
    case DivDir_SE:
    case DivDir_SW:
    case DivDir_ESE:
    case DivDir_WSW:
      l = down(l);
      break;
    default:
      break;
  }

  switch (divdirection)
  {
    case DivDir_ENE:
    case DivDir_ESE:
      l = right(l);
      // No break here is deliberate
    case DivDir_E:
    case DivDir_NE:
    case DivDir_SE:
    case DivDir_NNE:
    case DivDir_SSE:
      l = right(l);
      break;
    case DivDir_WNW:
    case DivDir_WSW:
      l = left(l);
      // No break here is deliberate
    case DivDir_W:
    case DivDir_NW:
    case DivDir_SW:
    case DivDir_NNW:
    case DivDir_SSW:
      l = left(l);
      break;
    default:
      break;
  }

  return l;
}

#endif
