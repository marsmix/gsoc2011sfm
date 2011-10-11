#include "Mesh.h"
#include "TracksOfPoints.h"

namespace OpencvSfM{

  Polygon::Polygon( )
  {
  }

  void Polygon::addPoint( const cv::Ptr< TrackOfPoints > point3D )
  {
    vertices.push_back( point3D );
  }

  Mesh::Mesh( )
  {
  }

  void Mesh::addPolygon( const cv::Ptr< Polygon > poly )
  {
    polygons.push_back( poly );
  }

}