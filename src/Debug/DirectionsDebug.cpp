#include "DirectionsDebug.hpp"

#include "../Graph/Directions.hpp"
#include "../Visualise/Printer.hpp"

void Debug::test_get_divdirection()
{
  for (int y = 0; y < 10; y++)
  {
    for (int x = 0; x < 10; x++)
    {
      if (x == 5 && y == 5)
      {
        std::cout << "X";
        continue;
      }
      auto curr_divdirection = get_divdirection_between_points(make_xyLoc(x,y), make_xyLoc(5,5));
      std::cout << int_to_drawn_char((int)curr_divdirection);
    }
    std::cout << "\n";
  }
  std::cout << std::flush;
}

