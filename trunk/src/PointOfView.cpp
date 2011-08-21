
#include "macro.h" //SFM_EXPORTS & remove annoying warnings

#include "opencv2/highgui/highgui.hpp"
#include "pcl/visualization/pcl_visualizer.h"
#include <iostream>
#include <pcl/io/vtk_io.h>
#include "opencv2/core/eigen.hpp"
#include "libmv/multiview/projection.h"

#include "PointOfView.h"

#include "TracksOfPoints.h"
#include "PCL_mapping.h"
#include "CameraPinholeDistor.h"

using cv::Mat;
using cv::Vec4d;
using cv::Vec3d;
using cv::Vec2d;
using cv::Vec;
using cv::Range;
using cv::Ptr;
using std::vector;
using std::string;
using cv::imread;
using libmv::Mat34;
using libmv::Mat3;
using libmv::Vec3;
using libmv::Vec;

namespace OpencvSfM{

  PointOfView::PointOfView( cv::Ptr<Camera> device, 
    cv::Mat rotation /*=Mat::eye( 3, 3, CV_64F )*/, 
    cv::Vec3d translation /*=Vec( 0.0,0.0,0.0 )*/ )
    : projection_matrix_( 3, 4, CV_64F )
  {
    CV_DbgAssert( rotation.rows==3 && rotation.cols==3 );
    CV_DbgAssert( !device.empty( ) );

    this->device_=device;

    this->rotation_=projection_matrix_( Range::all( ),Range( 0,3 ));
    rotation.copyTo( this->rotation_ );

    this->translation_ = projection_matrix_( Range::all( ),Range( 3,4 ));
    Mat( translation ).copyTo( this->translation_ );

    this->config_=0;//everything should be estimated...

    //as we are a new point of view related to a device, we should add our address into device_:
    device_->pointsOfView_.push_back( this );
  };


  PointOfView::PointOfView( cv::Mat projection_matrix )
    : projection_matrix_( 3, 4, CV_64F )
  {
    //convert to libmv matrix:
    libmv::Mat34 proj;
    cv::cv2eigen( projection_matrix, proj );
    libmv::Mat3 K, R;
    libmv::Vec3 t;
    libmv::KRt_From_P( proj, &K, &R, &t );
    
    if (R.determinant() < 0) {
      K.col(0) = -K.col(0);
      R.row(0) = -R.row(0);
      t(0)     = -t(0);
    }
    
    //enforce the rotation matrix.
    //TODO find the closest rotation matrix...
    Eigen::Quaterniond tmp = (Eigen::Quaterniond)R;
    tmp.normalize();
    R = tmp.toRotationMatrix();

    //create the device:
    cv::Mat K_cv;
    cv::eigen2cv( K, K_cv );
    device_ = cv::Ptr<Camera>( new CameraPinhole( K_cv ) );

    proj<<R,t;
    cv::eigen2cv( proj, projection_matrix_ );

    this->rotation_=projection_matrix_( Range::all( ),Range( 0,3 ));

    this->translation_ = projection_matrix_( Range::all( ),Range( 3,4 ));

    this->config_=0;//everything should be estimated...

    //as we are a new point of view related to a device, we should add our address into device_:
    device_->pointsOfView_.push_back( this );

  };

  PointOfView::~PointOfView( void )
  {
    this->projection_matrix_.release( );

    //remove the reference in device_->pointsOfView_:
    vector<PointOfView*>::iterator ourRef=device_->pointsOfView_.begin( );
    bool isFound=false;
    while( !isFound && ourRef != device_->pointsOfView_.end( ) )
    {
      //be aware of NULL pointor:
      if( ( *ourRef ) == this )
      {
        isFound=true;
        ( *ourRef )=NULL;
      }
      ourRef++;
    }
  }

  cv::Vec2d PointOfView::project3DPointIntoImage( cv::Vec3d point ) const
  {
    //As we don't know what type of camera we use ( with/without disportion, fisheyes... )
    //we can't use classic projection matrix P = K . [ R|t ]
    //Instead, we first compute points transformation into camera's system and then compute
    //pixel coordinate using camera device function.

    //As we need Mat object to compute projection, we create temporary objects:
    Mat mat3DNorm( 4,1,CV_64F );
    double* point3DNorm=( double* )mat3DNorm.data;
    Mat mat2DNorm( 3,1,CV_64F );
    double* point2DNorm=( double* )mat2DNorm.data;

    vector<Vec2d> pointsOut;
    point3DNorm[ 0 ] = point[ 0 ];
    point3DNorm[ 1 ] = point[ 1 ];
    point3DNorm[ 2 ] = point[ 2 ];
    point3DNorm[ 3 ] = 1;

    //transform points into camera's coordinates:
    mat2DNorm = ( projection_matrix_ * mat3DNorm );
    pointsOut.push_back( Vec2d( point2DNorm[ 0 ]/point2DNorm[ 2 ],point2DNorm[ 1 ]/point2DNorm[ 2 ] ));

    //transform points into pixel coordinates using camera intra parameters:
    pointsOut = device_->normImageToPixelCoordinates( pointsOut );
    Vec2d pointOut = pointsOut[ 0 ];
    return pointsOut[ 0 ];
  }
  std::vector< cv::Vec2d > PointOfView::project3DPointsIntoImage( 
    std::vector< cv::Vec3d > points ) const
  {
    //As we don't know what type of camera we use ( with/without disportion, fisheyes... )
    //we can't use classic projection matrix P = K . [ R|t ]
    //Instead, we first compute points transformation into camera's system and then compute
    //pixel coordinate using camera device function.

    //As we need Mat object to compute projection, we create temporary objects:
    Mat mat3DNorm( 4,1,CV_64F );
    double* point3DNorm=( double* )mat3DNorm.data;
    Mat mat2DNorm( 3,1,CV_64F );
    double* point2DNorm=( double* )mat2DNorm.data;
    
    vector<Vec2d> pointsOut;
    vector<Vec3d>::iterator point=points.begin( );
    while( point!=points.end( ) )
    {
      point3DNorm[ 0 ] = ( *point )[ 0 ];
      point3DNorm[ 1 ] = ( *point )[ 1 ];
      point3DNorm[ 2 ] = ( *point )[ 2 ];
      point3DNorm[ 3 ] = 1;

      //transform points into camera's coordinates:
      mat2DNorm = ( projection_matrix_ * mat3DNorm );

      pointsOut.push_back( Vec2d( point2DNorm[ 0 ]/point2DNorm[ 2 ],point2DNorm[ 1 ]/point2DNorm[ 2 ] ));

      point++;
    }
    //transform points into pixel coordinates using camera intra parameters:
    return device_->normImageToPixelCoordinates( pointsOut );
  }
  std::vector< cv::Vec2d > PointOfView::project3DPointsIntoImage(
    std::vector<TrackOfPoints> points ) const
  {
    //As we don't know what type of camera we use ( with/without disportion, fisheyes... )
    //we can't use classic projection matrix P = K . [ R|t ]
    //Instead, we first compute points transformation into camera's system and then compute
    //pixel coordinate using camera device function.

    //As we need Mat object to compute projection, we create temporary objects:
    Mat mat3DNorm( 4,1,CV_64F );
    double* point3DNorm=( double* )mat3DNorm.data;
    Mat mat2DNorm( 3,1,CV_64F );
    double* point2DNorm=( double* )mat2DNorm.data;
    
    vector<Vec2d> pointsOut;
    vector<TrackOfPoints>::iterator point=points.begin( );
    while( point!=points.end( ) )
    {
      cv::Ptr<Vec3d> convert_from_track = point->get3DPosition();
      if( !convert_from_track.empty() )
      {
        point3DNorm[ 0 ] = (*convert_from_track)[ 0 ];
        point3DNorm[ 1 ] = (*convert_from_track)[ 1 ];
        point3DNorm[ 2 ] = (*convert_from_track)[ 2 ];
        point3DNorm[ 3 ] = 1;

        //transform points into camera's coordinates:
        mat2DNorm = ( projection_matrix_ * mat3DNorm );

        pointsOut.push_back( Vec2d( point2DNorm[ 0 ]/point2DNorm[ 2 ],point2DNorm[ 1 ]/point2DNorm[ 2 ] ));
      }

      point++;
    }
    //transform points into pixel coordinates using camera intra parameters:
    return device_->normImageToPixelCoordinates( pointsOut );
  }

  bool PointOfView::pointInFrontOfCamera( cv::Vec4d point ) const
  {
    Mat pointTranspose= ( Mat ) point;
    double condition_1 = 
      this->projection_matrix_.row( 2 ).dot( pointTranspose.t( ) ) * point[ 3 ];
    double condition_2 = point[ 2 ] * point[ 3 ];
    if( condition_1 > 0 && condition_2 > 0 )
      return true;
    else
      return false;
  }
  cv::Mat PointOfView::getProjectionMatrix( ) const
  {
    return device_->getIntraMatrix( ) * projection_matrix_;
  };


  cv::Ptr<PointOfView> PointOfView::read( const cv::FileNode& node )
  {
    std::string myName=node.name( );
    if( myName != "PointOfView" )
    {
      std::string error = "FileNode is not correct!\nExpected \"PointOfView\", got ";
      error += node.name();
      CV_Error( CV_StsError, error.c_str() );
    }
    cv::FileNodeIterator it = node.begin();
    cv::Mat rot,trans;
    cv::Ptr<Camera> device;

    while( it != node.end() )
    {
      if((*it).name()=="rotation_")
        (*it)>>rot;
      if((*it).name()=="translation_")
        (*it)>>trans;
      if((*it).name()=="CameraPinhole")
        device = CameraPinhole::read( *it );
      if((*it).name()=="CameraPinholeDistor")
        device = CameraPinholeDistor::read( *it );
      it++;
    }
    return cv::Ptr<PointOfView>( new PointOfView(device, rot, trans) );
  }

  void PointOfView::write( cv::FileStorage& fs, const PointOfView& pov )
  {
    fs <<"{" << "PointOfView" <<"{" << "rotation_" << pov.rotation_;
    fs << "translation_" << pov.translation_;
    pov.device_->write( fs );
    fs << "}" << "}";
  }

}