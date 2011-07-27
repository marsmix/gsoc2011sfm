
#include "Visualizer.h"
#include "PCL_mapping.h"
#include <iostream>
#include <pcl/io/vtk_io.h>

using cv::Vec3d;
using cv::Mat;

namespace OpencvSfM{
  Visualizer::Visualizer( std::string name )
    :viewer( new pcl::visualization::PCLVisualizer ( name ) )
  {
    viewer->setBackgroundColor ( 0, 0, 0 );
    viewer->addCoordinateSystem ( 0.1 );
    viewer->initCameraParameters ( );
  }

  void Visualizer::addCamera( const PointOfView& camera,
    std::string name, int viewport )
  {
    //First get the focal of camera:
    const cv::Ptr<Camera> cam = camera.getIntraParameters( );
    double focal = cam->getFocal( );
    //Now get the corners in camera's coordinates:
    std::vector<cv::Vec2d> corners, real_coord;
    corners.push_back( cv::Vec2d( 0, 0 ) );
    corners.push_back( cv::Vec2d( 640, 480 ) );

    real_coord = cam->pixelToNormImageCoordinates( corners );

    //create the mesh:
    std::vector<Vec3d> camera_mesh;
    camera_mesh.push_back( Vec3d(
      real_coord[ 0 ][ 0 ], real_coord[ 0 ][ 1 ], 0 ) );//bottom left corner
    camera_mesh.push_back( Vec3d(
      real_coord[ 1 ][ 0 ], real_coord[ 0 ][ 1 ], 0 ) );//bottom right corner
    camera_mesh.push_back( Vec3d(
      real_coord[ 1 ][ 0 ], real_coord[ 1 ][ 1 ], 0 ) );//top right corner
    camera_mesh.push_back( Vec3d(
      real_coord[ 0 ][ 0 ], real_coord[ 1 ][ 1 ], 0 ) );//top left corner
    camera_mesh.push_back( Vec3d(
      0, 0, focal ) );//camera's origin

    //move theses points to the world coordinates:
    for( unsigned int i = 0; i < camera_mesh.size( ); ++i )
    {
      //translate to the correct location:
      camera_mesh[ i ] = ( Mat ) ( (Mat )( camera_mesh[ i ] ) - 
        camera.getTranslationVector( ) );
      //rotate the ith point:
      camera_mesh[ i ] = ( Mat ) ( (Mat )( camera_mesh[ i ] ).t( ) * 
        camera.getRotationMatrix( ) );
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud (
      new pcl::PointCloud<pcl::PointXYZ> );
    mapping::convert_OpenCV_vector( camera_mesh, *camera_cloud );

    //now create the mesh himself:
    std::vector<pcl::Vertices> vertices;
    pcl::Vertices vertice;
    vertice.vertices.push_back( 0 );
    vertice.vertices.push_back( 1 );
    vertice.vertices.push_back( 2 );
    vertice.vertices.push_back( 3 );
    vertices.push_back( vertice ); vertice.vertices.clear( );
    vertice.vertices.push_back( 0 );
    vertice.vertices.push_back( 1 );
    vertice.vertices.push_back( 4 );
    vertices.push_back( vertice ); vertice.vertices.clear( );
    vertice.vertices.push_back( 1 );
    vertice.vertices.push_back( 2 );
    vertice.vertices.push_back( 4 );
    vertices.push_back( vertice ); vertice.vertices.clear( );
    vertice.vertices.push_back( 2 );
    vertice.vertices.push_back( 3 );
    vertice.vertices.push_back( 4 );
    vertices.push_back( vertice ); vertice.vertices.clear( );
    vertice.vertices.push_back( 3 );
    vertice.vertices.push_back( 0 );
    vertice.vertices.push_back( 4 );
    vertices.push_back( vertice ); vertice.vertices.clear( );

    pcl::PolygonMesh triangles;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg( *camera_cloud, msg );
    triangles.cloud = msg;
    triangles.polygons = vertices;

    //pcl::io::saveVTKFile ( "mesh.vtk", triangles );
    //now add mesh to the viewer:
    viewer->addPolygonMesh<pcl::PointXYZ>( camera_cloud, vertices, name, viewport );
    viewer->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1.5, name );

    //viewer->addPointCloud<pcl::PointXYZ>( camera_cloud, name );
    //viewer->setPointCloudRenderingProperties (
    //  pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name );
  }
  void Visualizer::add3DPoints( const std::vector<cv::Vec3d>& points,
    std::string name, int viewport )
  {
    pcl::PointCloud< pcl::PointXYZ >::Ptr my_cloud (
      new pcl::PointCloud< pcl::PointXYZ > );
    mapping::convert_OpenCV_vector( points, *my_cloud );

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
      single_color( my_cloud, 0, 255, 0 );
    viewer->addPointCloud<pcl::PointXYZ> ( my_cloud, single_color,
      name, viewport );
    viewer->setPointCloudRenderingProperties (
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name, viewport );
    
    pcl::PolygonMesh triangles;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg( *my_cloud, msg );
    triangles.cloud = msg;
    pcl::io::saveVTKFile ( ((std::string)"test")+name+((std::string)".vtk"),
      triangles );
    
  }

  void Visualizer::runInteract( )
  {
    viewer->spin( );
  }
}