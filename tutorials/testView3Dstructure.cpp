
#include "config_SFM.h"
#include "../src/SequenceAnalyzer.h"
#include "../src/StructureEstimator.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>


#include "test_data_sets.h"
using namespace cv;
using namespace OpencvSfM;
using namespace OpencvSfM::tutorials;
using namespace std;

NEW_TUTO( Triangulation_tests, "Compute of 3D points from 2D position and cameras",
  "Using points projections and cameras parameters, try to find the 3D positions.\nThis is useful to test correctness of algorithms." ){

  int nviews = 5;
  int npoints = 6;

  cout<<"first load cameras and points..."<<endl;
  std::ifstream inPoints( FROM_SRC_ROOT( "Medias/Points_tests/logPoints.txt" ).c_str( ) );
  if( !inPoints.is_open( ) )
  {
    cout<<"This tuto need a file \"logPoints.txt\" with following syntax:"<<endl;
    cout<<"ProjectionMat0"<<endl;
    cout<<"K[ 0 ] K[ 1 ] K[ 2 ] K[ 3 ] K[ 4 ] K[ 5 ] K[ 6 ] K[ 7 ] K[ 8 ]"<<endl;
    cout<<"R[ 0 ] R[ 1 ] R[ 2 ] R[ 3 ] R[ 4 ] R[ 5 ] R[ 6 ] R[ 7 ] R[ 8 ]"<<endl;
    cout<<"T[ 0 ] T[ 1 ] T[ 2 ]"<<endl;
    cout<<"ProjectionMat1"<<endl;
    cout<<"K[ 0 ] K[ 1 ] K[ 2 ] K[ 3 ] K[ 4 ] K[ 5 ] K[ 6 ] K[ 7 ] K[ 8 ]"<<endl;
    cout<<"... ... ... ..."<<endl;
    cout<<"2D points"<<endl;
    cout<<"3D_pos[ 0 ] 3D_pos[ 1 ] 3D_pos[ 2 ]"<<endl;
    cout<<"2D_view_Cam1[ 0 ] 2D_view_Cam1[ 1 ]"<<endl;
    cout<<"2D_view_Cam2[ 0 ] 2D_view_Cam2[ 1 ]"<<endl;
    cout<<"2D_view_Cam3[ 0 ] 2D_view_Cam3[ 1 ]"<<endl;
    cout<<"... ... ... ..."<<endl;
    cout<<"2D points"<<endl;
    cout<<"3D_pos[ 0 ] 3D_pos[ 1 ] 3D_pos[ 2 ]"<<endl;
    cout<<"2D_view_Cam1[ 0 ] 2D_view_Cam1[ 1 ]"<<endl;
    cout<<"... ... ... ..."<<endl;
    cout<<"... ... ... ..."<<endl<<endl;

    cout<<"You can create any numbers of cameras you want"<<
      " and any numbers of 3D points, but be careful"<<
      " to have a 2D projection for each camera!"<<endl;
    cout<<"Please create this file before runing this tuto!"<<endl;
    return;
  }
  string skeepWord;

  // Collect P matrices together.
  vector<PointOfView> cameras;
  for ( int j = 0; j < nviews; ++j ) {
    cout<<"Loading a new camera..."<<endl;
    double R[ 9 ];
    double T[ 3 ];
    double K[ 9 ];
    inPoints >> skeepWord;
    while( skeepWord.find( "ProjectionsMat" )==std::string::npos && !inPoints.eof( ) )
      inPoints >> skeepWord;
    inPoints >> K[ 0 ] >> K[ 1 ] >> K[ 2 ];
    inPoints >> K[ 3 ] >> K[ 4 ] >> K[ 5 ];
    inPoints >> K[ 6 ] >> K[ 7 ] >> K[ 8 ];
    Ptr<Camera> device=Ptr<Camera>( new CameraPinhole( Mat( 3,3,CV_64F,K )) );
    inPoints >> R[ 0 ] >> R[ 1 ] >> R[ 2 ];
    inPoints >> R[ 3 ] >> R[ 4 ] >> R[ 5 ];
    inPoints >> R[ 6 ] >> R[ 7 ] >> R[ 8 ];
    inPoints >> T[ 0 ] >> T[ 1 ] >> T[ 2 ];
    //create the PointOfView:
    cameras.push_back( PointOfView( device, Mat( 3,3,CV_64F,R ), Vec3d( T ) ) );
  }

  vector<cv::Vec3d> points3D;
  vector< cv::Ptr< PointsToTrack > > points2D;
  vector< vector< cv::KeyPoint > > tracks_keypoints( nviews );
  for ( int i = 0; i < npoints; ++i ) {
    cout<<"Loading a new point ( and projections )..."<<endl;
    inPoints >> skeepWord;
    while( skeepWord != "2D" && !inPoints.eof( ) )
      inPoints >> skeepWord;
    inPoints >> skeepWord;//skip Point
    double X[ 3 ];
    inPoints >> X[ 0 ] >> X[ 1 ] >> X[ 2 ];
    points3D.push_back( Vec3d( X[ 0 ], X[ 1 ], X[ 2 ] ) );

    // Collect the image of point i in each frame.
    // inPoints >> skeepWord;
    for ( int j = 0; j < nviews; ++j ) {
      cv::Vec2f point;
      inPoints >> point[ 0 ] >> point[ 1 ];
      tracks_keypoints[ j ].push_back( KeyPoint( cv::Point2f( point[ 0 ], point[ 1 ] ),1 ) );
    }
  }
  for ( int j = 0; j < nviews; ++j ) {
    points2D.push_back( cv::Ptr<PointsToTrack>(
      new PointsToTrack( j, tracks_keypoints[ j ] ) ) );
  }

  Ptr<PointsMatcher> matches_algo ( new PointsMatcher(
    Ptr<DescriptorMatcher>( new FlannBasedMatcher( ) ) ) );
  SequenceAnalyzer motion_estim( points2D,matches_algo );

  //create tracks:
  vector<TrackOfPoints> tracks;
  for( int i=0; i<npoints; ++i )
  {
    TrackOfPoints tp;
    for( int j=0;j<nviews;j++ )
    {
      tp.addMatch( j,i );
    }
    tracks.push_back( tp );
  }
  motion_estim.addTracks( tracks );

  StructureEstimator structure ( &motion_estim, &cameras );
  tracks.clear( );
  vector<char> values_OK = structure.computeStructure( );

  tracks = motion_estim.getTracks( );
  //compute estimation error:
  double estim_error = 0.0;
  for( unsigned int it = 0; it < values_OK.size( ); ++it )
  {
    if( values_OK[ it ] != 0 )
    {
      cv::Vec3d& p3D = tracks[ it ];
      double distX = points3D[ it ][ 0 ] - p3D[ 0 ];
      double distY = points3D[ it ][ 1 ] - p3D[ 1 ];
      double distZ = points3D[ it ][ 2 ] - p3D[ 2 ];
      estim_error += sqrt( distX * distX + distY * distY + distZ * distZ );
    }
  }

  cout<<endl<<"Triangulation error: "<<estim_error<<endl<<endl;

}
