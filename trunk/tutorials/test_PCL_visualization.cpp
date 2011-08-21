#include "config_SFM.h"
#include "../src/PointsToTrackWithImage.h"
#include "../src/MotionProcessor.h"
#include "../src/StructureEstimator.h"
#include "../src/PointOfView.h"
#include "../src/CameraPinhole.h"
#include "../src/SequenceAnalyzer.h"
#include "../src/PCL_mapping.h"
#include "../src/Visualizer.h"

#include <boost/thread/thread.hpp>
#include <opencv2/calib3d/calib3d.hpp>



#include <sstream>
#include <numeric>

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//Only to see how we can create a 3D structure estimation using calibrated cameras
//////////////////////////////////////////////////////////////////////////

#include "test_data_sets.h"

#define POINT_METHOD "SIFT"

using namespace OpencvSfM;
using namespace OpencvSfM::tutorials;
using namespace std;
//using namespace cv;//these two namespaces overlaps!
//using namespace pcl;

NEW_TUTO( PCL_Tutorial, "Learn how you use PCL to show 3D points",
  "This tutorial will show you what you can see from the structure you extracted from motion." )
{
  //////////////////////////////////////////////////////////////////////////
  //first see if we have the points to show:
  SequenceAnalyzer *motion_estim_loaded;
  string pathFileTracks = FROM_SRC_ROOT( "Medias/motion_tracks_triangulated.yml" );
  std::ifstream test_file_exist;
  test_file_exist.open( pathFileTracks.c_str() );
  if( !test_file_exist.is_open( ) )
  {
    cout<<"you have to run an other tutorial before being able to run this one!"<<endl;
    bool worked = Tutorial_Handler::ask_to_run_tuto( "Triangulation_tuto" );
    if( !worked )
      return;
  }
  test_file_exist.close( );

  //////////////////////////////////////////////////////////////////////////
  //We also load cameras from the temple dataset:
  cout<<"Load the cameras from Medias/temple/temple_par.txt"<<endl;
  vector<PointOfView> myCameras=loadCamerasFromFile(
    FROM_SRC_ROOT( "Medias/temple/temple_par.txt" ));

  //////////////////////////////////////////////////////////////////////////
  //We can now load the points previously found:
  cout<<"create a SequenceAnalyzer using "<<pathFileTracks<<endl;
  cv::FileStorage fsRead( pathFileTracks, cv::FileStorage::READ );
  cv::FileNode myPtt = fsRead.getFirstTopLevelNode( );

  motion_estim_loaded = new SequenceAnalyzer( myPtt );
  fsRead.release( );
  SequenceAnalyzer::keepOnlyCorrectMatches( *motion_estim_loaded );
  cout<<"numbers of correct tracks loaded:"<<
    motion_estim_loaded->getTracks( ).size( )<<endl;

  //////////////////////////////////////////////////////////////////////////
  // Open 3D viewer and add point cloud

  Visualizer debugView ( "3D Viewer" );
  const vector< cv::Vec3d >& tracks = motion_estim_loaded->get3DStructure( );
  const vector< unsigned int > colors = motion_estim_loaded->getColors( );
  debugView.add3DPointsColored( tracks,colors, "Structure triangulated" );
  /*
  for( int i = 0; i<myCameras.size( ) ; ++i )
  {
    std::stringstream cam_name("Cam");
    cam_name<<i;
    debugView.addCamera( myCameras[ i ],
      cam_name.str() );
  }*/

  debugView.runInteract( );
}
