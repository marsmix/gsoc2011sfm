
#include "config_SFM.h"
#include "../src/PointsToTrackWithImage.h"
#include "../src/SequenceAnalyzer.h"
#include "../src/StructureEstimator.h"
#include "../src/EuclideanEstimator.h"

#include <opencv2/calib3d/calib3d.hpp>


//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////


#include "test_data_sets.h"
#define POINT_METHOD "SIFT"
using namespace cv;
using namespace OpencvSfM;
using namespace OpencvSfM::tutorials;
using namespace std;

NEW_TUTO( Proj_Rec, "Euclidean reconstruction",
  "Using points and intra parameters, try to compute motion and object" )
{

  //universal method to get the current image:
  vector<Mat> images;
  cout<<"Which cameras would you load?\n"
    "(0) Full temple dataset (>300 images)\n"
    "(1) Sparse temple dataset (16images)\n";
  int rep;
  cin>>rep;
  string inputDirectory,cameraFilename;
  if( rep==0 ){
    inputDirectory = FROM_SRC_ROOT( "Medias/temple/" );
    cameraFilename = inputDirectory + "temple_par.txt";
  }else{
    inputDirectory = FROM_SRC_ROOT( "Medias/templeSparseRing/" );
    cameraFilename = inputDirectory + "templeSR_par.txt";
  }

  MotionProcessor mp;
  mp.setInputSource( inputDirectory,IS_DIRECTORY );

  vector<PointOfView> myCameras =
    loadCamerasFromFile( cameraFilename, LOAD_INTRA );//LOAD_INTRA );
  vector<PointOfView> myCamerasReal =
    loadCamerasFromFile( cameraFilename, LOAD_FULL );
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

  string pathFileTracks = FROM_SRC_ROOT( "Medias/motion_track_no_error.yml" );
  std::ifstream inPoints( pathFileTracks.c_str( ) );
  if( !inPoints.is_open( ) )
  {
    inPoints.close( );
    pathFileTracks = FROM_SRC_ROOT( "Medias/tracks_points_"
      POINT_METHOD"/motion_tracks1.yml" );
    inPoints.open( pathFileTracks.c_str( ) );
    if( !inPoints.is_open( ) )
    {
      cout<<"you have to run an other tutorial before being able to run this one!"<<endl;
      bool worked = Tutorial_Handler::ask_to_run_tuto( "Track_creation" );
      if( !worked )
        return;
    }

    FileStorage fsRead( pathFileTracks, FileStorage::READ );
    FileNode myPtt = fsRead.getFirstTopLevelNode( );
    SequenceAnalyzer motion_estim_loaded( myPtt, &images,
      new PointsMatcher( DescriptorMatcher::create( "BruteForce-HammingLUT" ) ) );

    fsRead.release( );

    cout<<"A little help ;) Keep only good matches using triangulation."<<endl;
    StructureEstimator structure ( &motion_estim_loaded, &myCamerasReal );
    vector<char> mask =  structure.computeStructure( );
    vector<TrackOfPoints> &tracks=motion_estim_loaded.getTracks( );
    //remove bad tracks:
    for(unsigned int d = 0, d_idx=0;d<mask.size(); d++,d_idx++)
      if(mask[d]==0)
      {
        //remove this bad match:
        tracks[d_idx] = tracks[tracks.size()-1];
        d_idx--;
        tracks.pop_back();
      }
      structure.removeOutliersTracks( 2 );
      //save these correct points:
      pathFileTracks = FROM_SRC_ROOT( "Medias/motion_track_no_error.yml" );
      FileStorage fsOutMotion1( pathFileTracks, FileStorage::WRITE );
      SequenceAnalyzer::write( fsOutMotion1,motion_estim_loaded );
      fsOutMotion1.release( );
  }
  inPoints.close( );


  FileStorage fsRead( pathFileTracks, FileStorage::READ );
  FileNode myPtt = fsRead.getFirstTopLevelNode( );
  SequenceAnalyzer motion_estim_loaded( myPtt, &images,
    new PointsMatcher( DescriptorMatcher::create( "BruteForce-HammingLUT" ) ) );
  fsRead.release( );

  SequenceAnalyzer::keepOnlyCorrectMatches(motion_estim_loaded,2,0);
  vector<TrackOfPoints> &tracks=motion_estim_loaded.getTracks( );
  cout<<"numbers of correct tracks loaded:"<<tracks.size( )<<endl;

  //myCameras contains only intra value. I will use motion_estim_loaded to
  //compute position of cameras:
  EuclideanEstimator pe( motion_estim_loaded, myCameras );

  pe.computeReconstruction( );/*
  vector<int> idx_keep;
  for(int d = 0;d<myCameras.size(); d++){
    idx_keep.push_back(d);
    pe.camera_computed_[d] = true;
  }

  TrackOfPoints::keepTrackWithImages(idx_keep,tracks);
  pe.point_computed_ = tracks;
  pe.bundleAdjustement();
  pe.viewEstimation();
  //*/
}
