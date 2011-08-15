
#include "config_SFM.h"
#include "../src/PointsMatcher.h"
#include "../src/MotionProcessor.h"
#include "../src/CameraPinholeDistor.h"
#include "../src/StructureEstimator.h"
#include "../src/EuclideanEstimator.h"
#include "../src/Visualizer.h"

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//Problems with DISTORTION: unable to find a fast and accurate way to
//transform pixels in camera normalized coordinates into image pixels.
//The only way I found was to use LM iteration to find the original point...
//////////////////////////////////////////////////////////////////////////
#include "test_data_sets.h"

using namespace cv;
using namespace OpencvSfM;
using namespace OpencvSfM::tutorials;
using namespace std;

#define DEBUG_MESSAGE "To be able to run this tuto, please download model house dataset here :\n"\
"http://www.robots.ox.ac.uk/~vgg/data/data-mview.html"<<endl

NEW_TUTO( Model_House_test, "Using Model house data, run a SFM algorithm",
  "To be able to run this tuto, please download model house dataset here :\n"
  "http://www.robots.ox.ac.uk/~vgg/data/data-mview.html")
{

  cout<<"first load cameras and points from dataset..."<<endl;
  cout<<"Please set the datas in this directory: "<<
    FROM_SRC_ROOT( "Medias/modelHouse/*" )<<endl;
  std::ifstream inPoints( FROM_SRC_ROOT( "Medias/modelHouse/house.p3d" ).c_str( ) );
  if( !inPoints.is_open( ) )
  {
    cout<<DEBUG_MESSAGE<<"Needed file:  house.p3d..."<<endl;
    return;
  }
  //get 3D points:
  vector<cv::Vec3d> points;
  double pointsX,pointsY,pointsZ;
  while ( inPoints>>pointsX )
  {
    inPoints>>pointsY;
    inPoints>>pointsZ;
    points.push_back( cv::Vec3d(pointsX, pointsY, pointsZ) );
  }
  //the cameras:
  vector< PointOfView > cameras;
  std::ifstream inCams;
  int idFile = 0;
  stringstream nameFile;
  nameFile<<FROM_SRC_ROOT( "Medias/modelHouse/house.00" )<<idFile<<".P\0";
  inCams.open( nameFile.str().c_str( ) );
  double P[ 12 ];
  while( inCams.is_open() )
  {
    inCams >> P[ 0 ] >> P[ 1 ] >> P[ 2 ] >> P[ 3 ];
    inCams >> P[ 4 ] >> P[ 5 ] >> P[ 6 ] >> P[ 7 ];
    inCams >> P[ 8 ] >> P[ 9 ] >> P[10 ] >> P[11 ];
    //create the PointOfView:
    cameras.push_back( PointOfView( Mat( 3,4,CV_64F,P ) ) );

    idFile++;
    stringstream nameFile;
    nameFile<<FROM_SRC_ROOT( "Medias/modelHouse/house.00" )<<idFile<<".P\0";
    string tmpName= nameFile.str();
    inCams.close();
    inCams.open( tmpName.c_str( ) );
  }
  cout<<cameras.size()<<" cameras loaded!"<<endl;

  cout<<"Do you want to comput 3D points from matches (0) or use loaded points (1)?"<<endl;
  string rep;
  cin>>rep;
  if(rep!="1")
  {

    string pathFileTracks = FROM_SRC_ROOT( "Medias/modelHouse/motion.yml" );

    std::ifstream inPoints( pathFileTracks.c_str( ) );
    if( !inPoints.is_open( ) )
    {
      MotionProcessor mp;
      mp.setInputSource( FROM_SRC_ROOT( "Medias/modelHouse/" ),
        IS_DIRECTORY);
      SequenceAnalyzer motion_estim( mp,
        FeatureDetector::create("SIFT"),
        DescriptorExtractor::create("SIFT"),
        PointsMatcher::create("FlannBased") );

      cout<<"Compute matches..."<<endl;
      motion_estim.computeMatches();
      FileStorage fsOutMotion1( pathFileTracks, FileStorage::WRITE );
      if( !fsOutMotion1.isOpened( ) )
      {
        cout<<"Can't create "<<pathFileTracks<<"!\nPlease verify you have access to the directory!"<<endl;
        return;
      }
      //Can't find a way to enable the following notation:
      //fs << *ptt1;
      SequenceAnalyzer::write( fsOutMotion1,motion_estim );
      fsOutMotion1.release( );
    }
    inPoints.close( );

    FileStorage fsRead( pathFileTracks, FileStorage::READ );
    FileNode myPtt = fsRead.getFirstTopLevelNode( );
    vector<Mat> images;
    SequenceAnalyzer motion_estim( images, myPtt );
    fsRead.release( );


    //motion_estim.keepOnlyCorrectMatches( 3, 0 );

    vector<TrackOfPoints> &tracks=motion_estim.getTracks( );
    cout<<"numbers of correct tracks loaded:"<<tracks.size( )<<endl;

    cout<<"triangulation of points."<<endl;
    StructureEstimator structure ( motion_estim, cameras );
    vector<char> mask =  structure.computeStructure( );
    //remove bad points:
    for(unsigned int d = 0, d_idx=0;d<mask.size(); d++,d_idx++)
      if(mask[d]==0)
      {
        //remove this bad match:
        tracks[d_idx] = tracks[tracks.size()-1];
        d_idx--;
        tracks.pop_back();
      }

    cout<<"Bundle adjustement..."<<endl;
    EuclideanEstimator pe( motion_estim, cameras );

    for(unsigned int d = 0;d<cameras.size(); d++)
      pe.camera_computed_[d] = true;/*
    TrackOfPoints::keepTrackWithImage(0, tracks);
    TrackOfPoints::keepTrackWithImage(1, tracks);*/
    pe.point_computed_ = tracks;
    pe.bundleAdjustement();//test bundle adjustement....
    pe.viewEstimation();
  }
  else
  {
    Visualizer debugView ( "3D Viewer" );
    debugView.add3DPoints( points, "Structure triangulated" );
    for( unsigned int i = 0; i<cameras.size( ) ; ++i )
    {
      std::stringstream cam_name;
      cam_name<<"Cam"<< ( i+1 );
      debugView.addCamera( cameras[ i ],
        cam_name.str() );
    }

    debugView.runInteract();
  }
}
