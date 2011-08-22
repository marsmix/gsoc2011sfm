
#include "config_SFM.h"
#include "../src/MotionProcessor.h"
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
    cameras.push_back( PointOfView( Mat( 3, 4, CV_64F, P ) ) );

    idFile++;
    stringstream nameFile;
    nameFile<<FROM_SRC_ROOT( "Medias/modelHouse/house.00" )<<idFile<<".P\0";
    string tmpName= nameFile.str();
    inCams.close();
    inCams.open( tmpName.c_str( ) );
  }

  cout<<cameras.size()<<" cameras loaded!"<<endl;

  cout<<"Do you want to (0) compute 3D points using cameras ,\n"
    "(1) using points & cameras (noting computed)\n"
    " or (2) compute everything (except intra parameters)?"<<endl;
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
        FeatureDetector::create("FAST"),
        DescriptorExtractor::create("ORB"),
        PointsMatcher::create("BruteForce-HammingLUT") );

      cout<<"Compute matches..."<<endl;
      motion_estim.computeMatches();
      FileStorage fsOutMotion1( pathFileTracks, FileStorage::WRITE );
      if( !fsOutMotion1.isOpened( ) )
      {
        cout<<"Can't create "<<pathFileTracks<<"!\nPlease verify you have access to the directory!"<<endl;
        return;
      }
      SequenceAnalyzer::write( fsOutMotion1,motion_estim );
      fsOutMotion1.release( );
    }
    inPoints.close( );

    MotionProcessor mp;
    mp.setInputSource( FROM_SRC_ROOT( "Medias/modelHouse/" ),
      IS_DIRECTORY);
    vector<Mat> images;
    Mat imgTmp=mp.getFrame( );//get the current image
    while ( !imgTmp.empty() )
    {
      images.push_back( imgTmp );
      imgTmp=mp.getFrame();
    }

    FileStorage fsRead( pathFileTracks, FileStorage::READ );
    FileNode myPtt = fsRead.getFirstTopLevelNode( );
    SequenceAnalyzer motion_estim( myPtt, &images,
      new PointsMatcher( DescriptorMatcher::create( "BruteForce-Hamming" ) ) );
    fsRead.release( );

    SequenceAnalyzer::keepOnlyCorrectMatches( motion_estim, 3, 0 );
    vector<TrackOfPoints> &tracks=motion_estim.getTracks( );
    if(rep=="0")
    {
      cout<<"numbers of correct tracks loaded:"<<tracks.size( )<<endl;

      cout<<"triangulation of points."<<endl;
      StructureEstimator structure ( &motion_estim, &cameras );
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
        structure.removeOutliersTracks(2);
    }

    if(rep!="0")
    {
      //to be sure, remove rotation and translation:
      for(unsigned int d = 0;d<cameras.size(); d++)
      {
        cameras[d].setRotationMatrix( Mat::eye(3, 3, CV_64F) );
        cameras[d].setTranslationVector( Mat::zeros(3,1,CV_64F) );
      }
    }

    EuclideanEstimator pe( motion_estim, cameras );

    if(rep=="0")
    {
      for(unsigned int d = 0;d<cameras.size(); d++)
        pe.camera_computed_[d] = true;
      pe.point_computed_ = tracks;

      cout<<"Bundle adjustement..."<<endl;
      pe.bundleAdjustement();//test bundle adjustement...
      pe.viewEstimation( false );
    }
    else
    {
      //SequenceAnalyzer::keepOnlyCorrectMatches(motion_estim,3,0);
      pe.computeReconstruction();
      pe.viewEstimation( false );
    }
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
