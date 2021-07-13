#ifndef CORNERGRAPH_PROJECT__PRINTER_HPP
#define CORNERGRAPH_PROJECT__PRINTER_HPP

#include <string>
#include <vector>

#include "../Graph/XYLoc.hpp"

class Highlight
{
  private:
  unsigned int which_highlight;

  public:
  Highlight(const unsigned int input_which_highlight) : which_highlight(input_which_highlight) {}
  std::string to_string();
  bool is_no_highlighting() {return which_highlight == 0;}
  bool is_valid_highlighting() {return which_highlight >= 1 && which_highlight <= 13;}

  const static Highlight NO_HIGHLIGHTING;
};

class Printer
{
  private:
  int16_t width = 0;
  int16_t height = 0;
  std::vector<std::vector<char>> char_matrix;
  std::vector<std::vector<Highlight>> highlight_matrix;

  void expand_to_include_point(xyLoc l);

  public:
  void add_char(char c, xyLoc l);
  void add_highlight(Highlight h, xyLoc l);
  void print();
};

char int_to_drawn_char(int i);

#endif
