#include "RelevantPoints.hpp"


// Relevant: A corner is relevant to a pair of points if they are not safe reachable from each other, but c is safe-reachable from each point. 
//
// ###########---/-----
// ###########--/------
// ###########-/-------
// -----------c
// ----------/ 
// ---------/|
// --------/-|
// 
// If strictly to the left of the lower diagonal, then include all at top
//
// If in bottom triangle, then include top triangle
//
// Strictly to the left of lower diagonal are those that can safe-reach c with straight direction E and diagonal direction E or NE. 
//
// The bottom triangle are those that can safe-reach c with straight direction NE or N.

void RelevantPoints::get_relevant_points(corner_index i, DivDirection divdirection, const Graph &graph)
{
  
}

void RelevantPoints::get_relevant_corners(corner_index i, DivDirection divdirection)
{
}

void RelevantPoints::preprocess(const Graph &graph, const CornerVector &corner_vector, const NearbyCorners &nearby_corners)
{
  corner_to_divdirection_to_relevant_points.resize(corner_vector.size());
  corner_to_divdirection_to_relevant_corners.resize(corner_vector.size());
  for (corner_index i = 0; i < corner_vector.size(); i++)
  {
    if (i % 100 == 0)
      std::cout << i << ", " << std::flush;

    for (DivDirection divdirection : get_divdirections())
    {
      get_relevant_points(i, divdirection, graph);
      get_relevant_corners(i, divdirection);
    }
  }
  std::cout << '\n';
}
