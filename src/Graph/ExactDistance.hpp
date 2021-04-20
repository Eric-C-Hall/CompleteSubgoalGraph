#ifndef CORNERGRAPH_PROJECT__EXACT_DISTANCE_HPP
#define CORNERGRAPH_PROJECT__EXACT_DISTANCE_HPP

#define _USE_MATH_DEFINES
#include <cmath>

#include <climits>
#include <ostream>

struct exact_distance
{
  exact_distance() {}
  constexpr exact_distance(int num_straight_input, int num_diagonal_input) : num_straight(num_straight_input), num_diagonal(num_diagonal_input) {}

  int num_straight;
  int num_diagonal;
};

constexpr exact_distance MAX_EXACT_DISTANCE = exact_distance(INT_MAX, INT_MAX);
constexpr exact_distance STRAIGHT_EXACT_DISTANCE = exact_distance(1,0);
constexpr exact_distance DIAGONAL_EXACT_DISTANCE = exact_distance(0,1);
constexpr exact_distance ZERO_EXACT_DISTANCE = exact_distance(0,0);

inline bool operator<(const exact_distance &d1, const exact_distance &d2);
inline bool operator>(const exact_distance &d1, const exact_distance &d2);
inline bool operator<=(const exact_distance &d1, const exact_distance &d2);
inline bool operator>=(const exact_distance &d1, const exact_distance &d2);
inline bool operator==(const exact_distance &d1, const exact_distance &d2);
inline bool operator!=(const exact_distance &d1, const exact_distance &d2);

inline exact_distance operator+(const exact_distance &d1, const exact_distance &d2);
inline exact_distance operator-(const exact_distance &d1, const exact_distance &d2);
inline exact_distance operator*(const int multiplier, const exact_distance &d);

inline exact_distance operator+=(exact_distance &d1, const exact_distance &d2);

// definitions of inlines:

inline bool operator<(const exact_distance &d1, const exact_distance &d2)
{
  return d1.num_straight + d1.num_diagonal * M_SQRT2 < d2.num_straight + d2.num_diagonal * M_SQRT2;
}

inline bool operator>(const exact_distance &d1, const exact_distance &d2)
{
  return d1.num_straight + d1.num_diagonal * M_SQRT2 > d2.num_straight + d2.num_diagonal * M_SQRT2;
}

inline bool operator<=(const exact_distance &d1, const exact_distance &d2)
{
  return d1.num_straight + d1.num_diagonal * M_SQRT2 <= d2.num_straight + d2.num_diagonal * M_SQRT2;
}

inline bool operator>=(const exact_distance &d1, const exact_distance &d2)
{
  return d1.num_straight + d1.num_diagonal * M_SQRT2 >= d2.num_straight + d2.num_diagonal * M_SQRT2;
}

inline bool operator==(const exact_distance &d1, const exact_distance &d2)
{
  return d1.num_straight == d2.num_straight && d1.num_diagonal == d2.num_diagonal;
}

inline bool operator!=(const exact_distance &d1, const exact_distance &d2)
{
  return d1.num_straight != d2.num_straight || d1.num_diagonal != d2.num_diagonal;
}


inline exact_distance operator+(const exact_distance &d1, const exact_distance &d2)
{
  return exact_distance(d1.num_straight + d2.num_straight, d1.num_diagonal + d2.num_diagonal);
}

inline exact_distance operator-(const exact_distance &d1, const exact_distance &d2)
{
  return exact_distance(d1.num_straight - d2.num_straight, d1.num_diagonal - d2.num_diagonal);
}

inline exact_distance operator*(const int multiplier, const exact_distance &d)
{
  return exact_distance(multiplier * d.num_straight, multiplier * d.num_diagonal);
}

inline exact_distance operator+=(exact_distance &d1, const exact_distance &d2)
{
  d1.num_straight += d2.num_straight;
  d1.num_diagonal += d2.num_diagonal;
  return d1;
}

inline std::ostream& operator<<(std::ostream & stream, exact_distance dist)
{
  return stream << dist.num_straight << "s" << dist.num_diagonal << "d";
}

#endif
