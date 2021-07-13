#include "SaveLoad.hpp"

void SaveLoad::save_unsigned_int_as_binary(std::ostream & stream, unsigned int i)
{
  char * i_loc = (char *)&i;
  stream.write(i_loc, sizeof (unsigned int));
}

unsigned int SaveLoad::load_unsigned_int_as_binary(std::istream &stream)
{
  unsigned int return_value;
  stream.read((char *)(&return_value), sizeof (unsigned int));
  return return_value;
}

void SaveLoad::save_uint16_t_as_binary(std::ostream & stream, uint16_t i)
{
  char * i_loc = (char *)&i;
  stream.write(i_loc, sizeof (uint16_t));
}


int16_t SaveLoad::load_uint16_t_as_binary(std::istream &stream)
{
  uint16_t return_value;
  stream.read((char *)(&return_value), sizeof (uint16_t));
  return return_value;
}
