#include "Printer.hpp"

#include <cassert>
#include <cmath>

const Highlight Highlight::NO_HIGHLIGHTING = Highlight(0);

std::string Highlight::to_string()
{
  if (which_highlight == 0)
    return std::string("");

  if (which_highlight == 1)
    return std::string("\e[7m");

  if (which_highlight >= 2 && which_highlight <= 7)
    return std::string("\e[4") + std::to_string(which_highlight - 1) + std::string("m");

  if (which_highlight >= 8 && which_highlight <= 13)
    return std::string("\e[10") + std::to_string(which_highlight - 7) + std::string("m");

  return std::string("Error: invalid highlight ") + std::to_string(which_highlight);
}

void Printer::expand_to_include_point(xyLoc l)
{
  // Resize width if necessary
  if (l.x >= width)
  {
    width = l.x + 1;
    char_matrix.resize(width, std::vector<char>(height, '@'));
    highlight_matrix.resize(width, std::vector<Highlight>(height, Highlight::NO_HIGHLIGHTING));
  }

  // Resize height if necessary
  if (l.y >= height)
  {
    height = l.y + 1;
    for (auto &column : highlight_matrix)
      column.resize(height, Highlight::NO_HIGHLIGHTING);
    for (auto &column : char_matrix)
      column.resize(height, '@');
  }
}

void Printer::add_char(char c, xyLoc l)
{
  expand_to_include_point(l);
  char_matrix[l.x][l.y] = c;
}

void Printer::add_highlight(Highlight h, xyLoc l)
{
  expand_to_include_point(l);
  highlight_matrix[l.x][l.y] = h;
}

// The negative sign on negative numbers counts as a digit
int num_digits(int n)
{
  if (n == 0)
    return 1;

  if (n < 0)
    return num_digits(-n) + 1;

  int order = 0;

  while (std::pow(10, order) <= n)
  {
    order++;
  }

  return order;
}

void print_n_spaces(int n)
{
  while (n > 0)
  {
    std::cout << " ";
    n--;
  }
}

void print_sidebar(int y, unsigned int max_y)
{
  std::cout << y;

  int margin = num_digits(max_y) - num_digits(y) + 1;

  print_n_spaces(margin);
}

void print_topbar(const unsigned int width, const unsigned int max_y)
{
  print_n_spaces(num_digits(max_y) + 1);

  int i = 0;
  while ((int)width - i >= (int)num_digits(i))
  {
    std::cout << i;
    i += num_digits(i);
    std::cout << " ";
    i += 1;
  }
  std::cout << "\n";
}

void Printer::print()
{
  print_topbar(width, height-1);

  for (int y = 0; y < height; y++)
  {
    print_sidebar(y, height-1);
    for (int x = 0; x < width; x++)
    {
      assert ((int16_t)highlight_matrix.size() > x);
      assert ((int16_t)highlight_matrix[x].size() > y);
      Highlight h = highlight_matrix[x][y];

      assert((int16_t)char_matrix.size() > x);
      assert((int16_t)char_matrix[x].size() > y);
      char c = char_matrix[x][y];
      
      if (!h.is_no_highlighting())
        std::cout << h.to_string();

      std::cout << c;

      if (!h.is_no_highlighting())
        std::cout << "\e[0m";
    }
    std::cout << '\n';
  }
  std::cout << std::flush;
}

char int_to_drawn_char(int i)
{
  if (i < -1)
  {
    return '<';
  }
  if (i == -1)
  {
    return '~';
  }
  else if (i <= 9)
  {
    return '0' + i;
  }
  else if (i <= 10 + 25)
  {
    return 'a' + i - 10;
  }
  else if (i <= 10 + 26 + 25)
  {
    return 'A' + i - (10 + 26);
  }
  else
  {
    return '>';
  }
}
