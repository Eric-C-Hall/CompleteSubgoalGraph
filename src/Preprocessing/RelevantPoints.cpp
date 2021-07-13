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

void get_relevant_points(corner_index i, DivDirection middirection, const Graph &graph)
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

    for (MidDirection middirection : get_middirections())
    {
      get_relevant_points(i, middirection, graph);
      get_relevant_corners(i, middirection);
    }
  }
  std::cout << '\n';
}
