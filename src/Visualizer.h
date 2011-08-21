#ifndef _GSOC_SFM_VISUALIZER_H
#define _GSOC_SFM_VISUALIZER_H 1

#include "macro.h" //SFM_EXPORTS

#include "opencv2/core/core.hpp"
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>

namespace OpencvSfM{
  class SFM_EXPORTS PointOfView;
  /**
  * @brief This class can be used to view the differents object involved in current
  * structure from motion process.
  * 
  * You can add to visualization 3D points, cameras, pictures...
  * This class use PCL as back end, but it's hidden!
  */
  class SFM_EXPORTS Visualizer{
  protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;///<The PCL viewer
  public:
    /**
    * Use this constructor to create a new window
    * @param name The title of the new window
    */
    Visualizer( std::string name = "3D Viewer" );
    
    /**
    * Use this function to add a new camera to the visualizer
    * @param camera info about the wanted camera
    * @param name The name of the printed object
    * @param viewport idx of the wanted viewport
    */
    void addCamera( const PointOfView& camera,
      std::string name = "camera", int viewport = 0 );

    /**
    * Use this function to add a new point cloud to the visualizer
    * @param points list of 3d points
    * @param name The name of the printed object
    * @param viewport idx of the wanted viewport
    */
    void add3DPoints( const std::vector<cv::Vec3d>& points,
      std::string name = "cloud", int viewport = 0 );
    
    /**
    * Use this function to add a new point cloud with color to the visualizer
    * @param points list of 3d points
    * @param colors list of colors (RGB packed)
    * @param name The name of the printed object
    * @param viewport idx of the wanted viewport
    */
    void add3DPointsColored( const std::vector<cv::Vec3d>& points,
      const std::vector<unsigned int>& colors,
      std::string name = "cloud", int viewport = 0 );
    
    /**
    * Once geometry is added, you can used this function to enable user interaction
    */
    void runInteract( );
  };
}

#endif