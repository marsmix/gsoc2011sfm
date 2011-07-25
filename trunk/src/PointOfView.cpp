#include "PointOfView.h"

#include "libmv_mapping.h"
#include "PCL_mapping.h"
#include <iostream>
#include <pcl/io/vtk_io.h>

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

namespace OpencvSfM{

  PointOfView::PointOfView(cv::Ptr<Camera> device, Mat rotation /*=Mat::eye(3, 3, CV_64F)*/, Vec3d translation /*=Vec(0.0,0.0,0.0)*/ )
    : projection_matrix_(3, 4, CV_64F)
  {
    CV_DbgAssert( rotation.rows==3 && rotation.cols==3 );
    CV_DbgAssert( !device.empty() );

    this->device_=device;

    this->rotation_=projection_matrix_(Range::all(),Range(0,3));
    rotation.copyTo( this->rotation_ );

    this->translation_ = projection_matrix_(Range::all(),Range(3,4));
    Mat(translation).copyTo( this->translation_ );

    this->config_=0;//everything should be estimated...

    //as we are a new point of view related to a device, we should add our address into device_:
    device_->pointsOfView_.push_back(this);
  };

  PointOfView::~PointOfView(void)
  {
    this->projection_matrix_.release();

    //remove the reference in device_->pointsOfView_:
    vector<PointOfView*>::iterator ourRef=device_->pointsOfView_.begin();
    bool isFound=false;
    while(!isFound && ourRef != device_->pointsOfView_.end())
    {
      //be aware of NULL pointor:
      if( (*ourRef) == this )
      {
        isFound=true;
        (*ourRef)=NULL;
      }
      ourRef++;
    }
  }

  Vec2d PointOfView::project3DPointIntoImage(Vec3d point) const
  {
    //As we don't know what type of camera we use (with/without disportion, fisheyes...)
    //we can't use classic projection matrix P = K . [R|t]
    //Instead, we first compute points transformation into camera's system and then compute
    //pixel coordinate using camera device function.

    //As we need Mat object to compute projection, we create temporary objects:
    Mat mat3DNorm(4,1,CV_64F);
    double* point3DNorm=(double*)mat3DNorm.data;
    Mat mat2DNorm(3,1,CV_64F);
    double* point2DNorm=(double*)mat2DNorm.data;

    vector<Vec2d> pointsOut;
    point3DNorm[0] = point[0];
    point3DNorm[1] = point[1];
    point3DNorm[2] = point[2];
    point3DNorm[3] = 1;

    //transform points into camera's coordinates:
    mat2DNorm = ( projection_matrix_ * mat3DNorm);
    pointsOut.push_back(Vec2d(point2DNorm[0]/point2DNorm[2],point2DNorm[1]/point2DNorm[2]));

    //transform points into pixel coordinates using camera intra parameters:
    pointsOut = device_->normImageToPixelCoordinates(pointsOut);
    return pointsOut[0];
  }
  vector<Vec2d> PointOfView::project3DPointsIntoImage(vector<Vec3d> points) const
  {
    //As we don't know what type of camera we use (with/without disportion, fisheyes...)
    //we can't use classic projection matrix P = K . [R|t]
    //Instead, we first compute points transformation into camera's system and then compute
    //pixel coordinate using camera device function.

    //As we need Mat object to compute projection, we create temporary objects:
    Mat mat3DNorm(4,1,CV_64F);
    double* point3DNorm=(double*)mat3DNorm.data;
    Mat mat2DNorm(3,1,CV_64F);
    double* point2DNorm=(double*)mat2DNorm.data;
    
    vector<Vec2d> pointsOut;
    vector<Vec3d>::iterator point=points.begin();
    while(point!=points.end())
    {
      point3DNorm[0] = (*point)[0];
      point3DNorm[1] = (*point)[1];
      point3DNorm[2] = (*point)[2];
      point3DNorm[3] = 1;

      //transform points into camera's coordinates:
      mat2DNorm = ( projection_matrix_ * mat3DNorm);

      pointsOut.push_back(Vec2d(point2DNorm[0]/point2DNorm[2],point2DNorm[1]/point2DNorm[2]));

      point++;
    }
    //transform points into pixel coordinates using camera intra parameters:
    return device_->normImageToPixelCoordinates(pointsOut);
  }
  vector<Vec2d> PointOfView::project3DPointsIntoImage(vector<TrackOfPoints> points) const
  {
    //As we don't know what type of camera we use (with/without disportion, fisheyes...)
    //we can't use classic projection matrix P = K . [R|t]
    //Instead, we first compute points transformation into camera's system and then compute
    //pixel coordinate using camera device function.

    //As we need Mat object to compute projection, we create temporary objects:
    Mat mat3DNorm(4,1,CV_64F);
    double* point3DNorm=(double*)mat3DNorm.data;
    Mat mat2DNorm(3,1,CV_64F);
    double* point2DNorm=(double*)mat2DNorm.data;
    
    vector<Vec2d> pointsOut;
    vector<TrackOfPoints>::iterator point=points.begin();
    while(point!=points.end())
    {
      Vec3d convert_from_track = (*point);
      point3DNorm[0] = convert_from_track[0];
      point3DNorm[1] = convert_from_track[1];
      point3DNorm[2] = convert_from_track[2];
      point3DNorm[3] = 1;

      //transform points into camera's coordinates:
      mat2DNorm = ( projection_matrix_ * mat3DNorm);

      pointsOut.push_back(Vec2d(point2DNorm[0]/point2DNorm[2],point2DNorm[1]/point2DNorm[2]));

      point++;
    }
    //transform points into pixel coordinates using camera intra parameters:
    return device_->normImageToPixelCoordinates(pointsOut);
  }

  bool PointOfView::pointInFrontOfCamera(cv::Vec4d point) const
  {
    Mat pointTranspose= (Mat) point;
    double condition_1 = this->projection_matrix_.row(2).dot(pointTranspose.t()) * point[3];
    double condition_2 = point[2] * point[3];
    if( condition_1 > 0 && condition_2 > 0 )
      return true;
    else
      return false;
  }
  cv::Mat PointOfView::getProjectionMatrix() const
  {
    return device_->getIntraMatrix() * projection_matrix_;
  };

  void PointOfView::addCameraRepresentation(
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    int image_width, int image_height, std::string name, int viewport )
  {
    //First get the focal of camera:
    double focal = device_->getFocal();
    //Now get the corners in camera's coordinates:
    std::vector<cv::Vec2d> corners, real_coord;
    corners.push_back( cv::Vec2d( 0, 0 ) );
    corners.push_back( cv::Vec2d( image_width, image_height ) );

    real_coord = device_->pixelToNormImageCoordinates( corners );

    //create the mesh:
    std::vector<Vec3d> camera_mesh;
    camera_mesh.push_back( Vec3d(
      real_coord[0][0], real_coord[0][1], 0) );//bottom left corner
    camera_mesh.push_back( Vec3d(
      real_coord[1][0], real_coord[0][1], 0) );//bottom right corner
    camera_mesh.push_back( Vec3d(
      real_coord[1][0], real_coord[1][1], 0) );//top right corner
    camera_mesh.push_back( Vec3d(
    real_coord[0][0], real_coord[1][1], 0) );//top left corner
    camera_mesh.push_back( Vec3d(
      0, 0, focal ) );//camera's origin

    //move theses points to the world coordinates:
    for(unsigned int i = 0; i < camera_mesh.size(); ++i)
    {
      //translate to the correct location:
      camera_mesh[i] = (Mat) ((Mat)(camera_mesh[i]) - translation_);
      //rotate the ith point:
      camera_mesh[i] = (Mat) ((Mat)(camera_mesh[i]).t() * rotation_ );
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud (
      new pcl::PointCloud<pcl::PointXYZ>);
    mapping::convert_OpenCV_vector( camera_mesh, *camera_cloud );

    //now create the mesh himself:
    std::vector<pcl::Vertices> vertices;
    pcl::Vertices vertice;
    vertice.vertices.push_back(0);
    vertice.vertices.push_back(1);
    vertice.vertices.push_back(2);
    vertice.vertices.push_back(3);
    vertices.push_back( vertice ); vertice.vertices.clear();
    vertice.vertices.push_back(0);
    vertice.vertices.push_back(1);
    vertice.vertices.push_back(4);
    vertices.push_back( vertice ); vertice.vertices.clear();
    vertice.vertices.push_back(1);
    vertice.vertices.push_back(2);
    vertice.vertices.push_back(4);
    vertices.push_back( vertice ); vertice.vertices.clear();
    vertice.vertices.push_back(2);
    vertice.vertices.push_back(3);
    vertice.vertices.push_back(4);
    vertices.push_back( vertice ); vertice.vertices.clear();
    vertice.vertices.push_back(3);
    vertice.vertices.push_back(0);
    vertice.vertices.push_back(4);
    vertices.push_back( vertice ); vertice.vertices.clear();

    pcl::PolygonMesh triangles;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg( *camera_cloud, msg);
    triangles.cloud = msg;
    triangles.polygons = vertices;

    //pcl::io::saveVTKFile ("mesh.vtk", triangles);
    //now add mesh to the viewer:
    viewer->addPolygonMesh<pcl::PointXYZ>(camera_cloud, vertices, name, viewport);
    //viewer->setShapeRenderingProperties(
    //  pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, name);

    viewer->addPointCloud<pcl::PointXYZ>(camera_cloud, name);
    viewer->setPointCloudRenderingProperties (
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
  }
}