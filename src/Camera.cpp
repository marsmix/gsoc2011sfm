#include "Camera.h"

using cv::Mat;
using cv::Vec3d;
using cv::Range;
using std::vector;

namespace OpencvSfM{

  Camera::Camera( int img_w, int img_h )
  {
    img_width = img_w;
    img_height = img_h;
  }


  Camera::~Camera( void )
  {
  }

  cv::Ptr<Camera> Camera::read( const cv::FileNode& node )
  {
    std::string myName=node.name( );
    if( myName != "Camera" )
    {
      std::string error = "Camera FileNode is not correct!\nExpected \"Camera\", got ";
      error += node.name();
      CV_Error( CV_StsError, error.c_str() );
    }
    //nothing to do as we are a fake camera...
    return cv::Ptr<Camera>( NULL );
  }

}