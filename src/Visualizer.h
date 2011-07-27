#ifndef _GSOC_SFM_VISUALIZER_H
#define _GSOC_SFM_VISUALIZER_H 1

#include "macro.h" //SFM_EXPORTS
#include "PointOfView.h"

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace OpencvSfM{

  /**
  * @brief This class can be used to view the differents object involved in current
  * structure from motion process.
  * 
  * You can add to visualization 3D points, cameras, pictures...
  * This class use PCL as back end, but it's hidden!
  */
  class SFM_EXPORTS Visualizer{
  protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  public:
    Visualizer( std::string name = "3D Viewer" );
    
    void addCamera( const PointOfView& camera,
      std::string name = "camera", int viewport = 0 );
    void add3DPoints( const std::vector<cv::Vec3d>& points,
      std::string name = "cloud", int viewport = 0 );

    void runInteract( );
  };
}

#endif