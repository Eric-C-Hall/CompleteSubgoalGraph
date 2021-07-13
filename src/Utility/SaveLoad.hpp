#ifndef CORNERGRAPH_PROJECT__SAVE_LOAD_HPP
#define CORNERGRAPH_PROJECT__SAVE_LOAD_HPP

#include <cstdint>
#include <iostream>

namespace SaveLoad
{
  void save_unsigned_int_as_binary(std::ostream & stream, unsigned int i);
  unsigned int load_unsigned_int_as_binary(std::istream &stream);

  void save_uint16_t_as_binary(std::ostream & stream, uint16_t i);
  int16_t load_uint16_t_as_binary(std::istream &stream);
}

#endif
