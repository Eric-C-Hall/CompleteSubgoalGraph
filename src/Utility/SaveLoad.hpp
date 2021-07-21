#ifndef CORNERGRAPH_PROJECT__SAVE_LOAD_HPP
#define CORNERGRAPH_PROJECT__SAVE_LOAD_HPP

#include <cstdint>
#include <iostream>

namespace SaveLoad
{
  template<class T>
  void save_as_binary(std::ostream & stream, const T &t);

  template<class T>
  void load_as_binary(std::istream &stream, T &t);
}

template<class T>
void SaveLoad::save_as_binary(std::ostream & stream, const T &t)
{
  char * t_loc = (char *)&t;
  stream.write(t_loc, sizeof (T));
}

template<class T>
void SaveLoad::load_as_binary(std::istream &stream, T &t)
{
  stream.read((char *)(&t), sizeof (T));
}

#endif
