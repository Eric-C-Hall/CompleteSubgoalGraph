#include <iostream>


int main(int argc, char **argv)
{
  int SQRT_2_LESS_PRECISION = 141421;
  int SQRT_2_MORE_PRECISION = 1414214;

  for (int i = 0; ;i++)
  {
    int num_straight_less_precision = (i * SQRT_2_LESS_PRECISION) / 100000;
    int num_straight_more_precision = (i * SQRT_2_MORE_PRECISION) / 1000000;
    if (num_straight_less_precision < num_straight_more_precision)
    {
      std::cout << i << " * " << SQRT_2_LESS_PRECISION << " = " << (i * SQRT_2_LESS_PRECISION) << " < " << num_straight_less_precision << " * 100000" << std::endl;
      std::cout << i << " * " << SQRT_2_MORE_PRECISION << " = " << (i * SQRT_2_MORE_PRECISION) << " > " << num_straight_more_precision << " * 1000000" << std::endl;
      break;
    }
  }

  return 0;
}
