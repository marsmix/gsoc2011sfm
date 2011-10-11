
#ifndef _GSOC_SFM_MESH_H
#define _GSOC_SFM_MESH_H 1

#include "macro.h" //SFM_EXPORTS

#include <vector>
#include "opencv2/core/core.hpp"

namespace OpencvSfM{
  class SFM_EXPORTS TrackOfPoints;
  
  /*! \brief This class is used to handle a facet of a mesh
  */
  class SFM_EXPORTS Polygon
  {
  protected:
    std::vector< cv::Ptr< TrackOfPoints > > vertices;///<list of points

  public:
    /**
    * Constructor
    */
    Polygon();

    /**
    * Use this function to add a point to the polygon
    * @param point3D address of point who's copied into the vertices list
    */
    void addPoint( const cv::Ptr< TrackOfPoints > point3D );
  };

  
  /*! \brief This class is used to handle a entire mesh
  */
  class SFM_EXPORTS Mesh
  {
  protected:
    std::vector< cv::Ptr< Polygon > > polygons;///<list of polygons

  public:
    /**
    * Constructor
    */
    Mesh();
    
    /**
    * Use this function to add a polygon to the mesh
    * @param poly address of polygon who's copied into the polygons list
    */
    void addPolygon( const cv::Ptr< Polygon > poly );
  };
}

#endif