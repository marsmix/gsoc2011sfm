
#include "config_SFM.h"
#include "../src/StructureEstimator.h"
#include "../src/PointOfView.h"
#include "../src/SequenceAnalyzer.h"

#include <boost/thread/thread.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "pcl/visualization/pcl_visualizer.h"

#include <numeric>

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//Only to see how we can create a 3D structure estimation using calibrated cameras
//////////////////////////////////////////////////////////////////////////

#include "test_data_sets.h"
using namespace cv;
using namespace OpencvSfM;
using namespace OpencvSfM::tutorials;
using namespace std;

#define POINT_METHOD "SIFT"

NEW_TUTO( Triangulation_tuto, "Learn how you can triangulate 2D points",
  "Using fully parameterized cameras, we find 2D points in the sequence and then triangulate them." )
{
  vector<Mat> images;

  SequenceAnalyzer *motion_estim_loaded;
  stringstream pathFile;
  pathFile<<FROM_SRC_ROOT( "Medias/" );
  cout<<"Which file would you open? Usually :"<<endl<<"tracks_points_"POINT_METHOD"/motion_tracks.yml"<<endl;
  string nameFile;
  cin>>nameFile;
  pathFile<<nameFile;
  string pathFileTracks = pathFile.str();
  std::ifstream test_file_exist;
  test_file_exist.open( pathFileTracks.c_str() );
  if( !test_file_exist.is_open( ) )
  {
    cout<<"you should run an other tutorial before being able to run this one!"<<endl;
    bool worked = Tutorial_Handler::ask_to_run_tuto( "Track_creation" );
    if( !worked )
      return;
    pathFileTracks = FROM_SRC_ROOT( "Medias/tracks_points_"POINT_METHOD"/motion_tracks.yml" );
  }
  test_file_exist.close( );

  cout<<"First load the cameras from Medias/temple/temple_par.txt"<<endl;
  vector<PointOfView> myCameras=loadCamerasFromFile( FROM_SRC_ROOT( "Medias/temple/temple_par.txt" ));
  MotionProcessor mp;
  mp.setInputSource( FROM_SRC_ROOT( "Medias/temple/" ),IS_DIRECTORY );

  cout<<"Then load all images from Medias/temple/"<<endl;
  vector<PointOfView>::iterator itPoV=myCameras.begin( );
  int index_image=-1;
  while ( itPoV!=myCameras.end( ) )
  {
    Mat imgTmp=mp.getFrame( );//get the current image
    if( imgTmp.empty( ) )
      break;//end of sequence: quit!
    index_image++;
    images.push_back( imgTmp );
  }

  cout<<"Finally create a new SequenceAnalyzer"<<endl;
  FileStorage fsRead( pathFileTracks, FileStorage::READ );
  FileNode myPtt = fsRead.getFirstTopLevelNode( );
  motion_estim_loaded = new SequenceAnalyzer( images, myPtt );
  fsRead.release( );

  cout<<"numbers of correct tracks loaded:"<<
    motion_estim_loaded->getTracks( ).size( )<<endl;

  int maxImg=motion_estim_loaded->getNumViews( );

  cout<<"triangulation of points."<<endl;
  StructureEstimator structure ( motion_estim_loaded, &myCameras );
  vector<char> mask =  structure.computeStructure( );
  cout<<std::accumulate( mask.begin( ), mask.end( ), 0 )<<" 3D points found."<<endl;

  //now save the triangulate tracks:
  pathFileTracks = FROM_SRC_ROOT( "Medias/motion_tracks_triangulated.yml" );
  FileStorage fsOutMotion( pathFileTracks, FileStorage::WRITE );
  if( !fsOutMotion.isOpened( ) )
    CV_Error( 0,"Can't create the file!\nPlease verify you have access to the directory!" );

  SequenceAnalyzer::write( fsOutMotion, *motion_estim_loaded );
  fsOutMotion.release( );
}
