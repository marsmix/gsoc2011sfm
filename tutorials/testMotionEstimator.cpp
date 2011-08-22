
#include "config_SFM.h"
#include "../src/PointsToTrackWithImage.h"
#include "../src/MotionProcessor.h"
#include "../src/SequenceAnalyzer.h"
#include "../src/PointsMatcher.h"
//#include "../src/libmv_mapping.h"
#include <opencv2/calib3d/calib3d.hpp>

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

#include "test_data_sets.h"
//FeatureDetector::create( "FAST" )
#define POINT_METHOD "SIFT"
#define POINT_DETECTOR FeatureDetector::create( "FAST" )
#define DESCRIPTOR_METHOD "ORB"
#define MATCHER_METHOD "BruteForce-Hamming"

using namespace cv;
using namespace OpencvSfM;
using namespace OpencvSfM::tutorials;
using namespace std;

NEW_TUTO( Track_creation, "Learn how you can compute tracks from a list of pictures",
  "Using features detector and matcher, create a vector of tracks ( a track is a list of 2D points from the same 3D point ).\nYou will also learn how you can save an object using YAML!" )
{
  cout<<"Which dataset would you load?\n"
    "(0) Full temple dataset (>300 images)\n"
    "(1) Sparse temple dataset (16images)\n";
  int rep;
  cin>>rep;
  string inputDirectory;
  if( rep==0 )
    inputDirectory = FROM_SRC_ROOT( "Medias/temple/" );
  else
    inputDirectory = FROM_SRC_ROOT( "Medias/templeSparseRing/" );
  MotionProcessor mp;
  //first load images:
  //Here we will a folder with a lot of images, but we can do the same thing with any other type of input
  if( !mp.setInputSource( inputDirectory,IS_DIRECTORY ) )
  {
    cout<<"Can't find dataset"<<endl;
    return;
  }

  //Configure input ( not needed, but show how we can do
  //mp.setProperty( CV_CAP_PROP_CONVERT_RGB,0 );//Only greyscale, due to SIFT
  mp.setProperty( CV_CAP_PROP_CONVERT_RGB,0 );//convert to grayscal

  Ptr<FeatureDetector> detector = POINT_DETECTOR;
  Ptr<DescriptorExtractor> desc_extractor;
  desc_extractor = DescriptorExtractor::create( DESCRIPTOR_METHOD );
  vector< Ptr< PointsToTrack > > vec_points_to_track;
  Ptr<PointsToTrack> ptrPoints_tmp;

  Ptr<DescriptorMatcher> matcher;
  matcher = DescriptorMatcher::create( MATCHER_METHOD );
  Ptr<PointsMatcher> matches_algo ( new PointsMatcher( matcher ) );

  //universal method to get the current image:
  vector<Mat> images;
  Mat currentImage=mp.getFrame( );

  string pathFileTracks = FROM_SRC_ROOT( "Medias/tracks_points_"POINT_METHOD"/motion_points.yml" );
  std::ifstream inPoints( pathFileTracks.c_str( ) );
  if( inPoints.is_open( ) )
  {
    inPoints.close( );
    cout<<"Load points from \"motion_points.yml\""<<endl;
    FileStorage fsRead( pathFileTracks, FileStorage::READ );
    FileNodeIterator vecPtt = fsRead.getFirstTopLevelNode( ).begin( );
    FileNodeIterator vecPtt_end = fsRead.getFirstTopLevelNode( ).end( );

    while( vecPtt != vecPtt_end && !currentImage.empty( ) )
    {
      Ptr<PointsToTrack> ptt = Ptr<PointsToTrack>( new PointsToTrack( ) );
      PointsToTrack::read( ( *vecPtt )[ 0 ][ "PointsToTrack" ],*ptt );
      vec_points_to_track.push_back( ptt );
      images.push_back( currentImage );
      currentImage=mp.getFrame( );
      vecPtt++;
      cout<<"|";
    }
    cout<<endl;

    fsRead.release( );
  }
  else
  {
    int nbFrame=0;
    cout<<"Compute points and description for each frame..."<<endl;
    cout<<"This can take time so be patient ;)"<<endl;
    while ( !currentImage.empty( ) )
    {
      //if the image is loaded, find the points:
      cout<<"Create a new PointsToTrack..."<<endl;

      ptrPoints_tmp = Ptr<PointsToTrack>( new PointsToTrackWithImage ( nbFrame,
        currentImage, detector, desc_extractor ));
      ptrPoints_tmp->computeKeypointsAndDesc( );

      vec_points_to_track.push_back( ptrPoints_tmp );
      images.push_back( currentImage );
      nbFrame++;
      currentImage=mp.getFrame( );
    }

    //now save the tracks:
    FileStorage fsOut( pathFileTracks, FileStorage::WRITE );
    if( !fsOut.isOpened( ) )
    {
      cout<<"Can't create "<<pathFileTracks<<"!\nPlease verify you have access to the directory!"<<endl;
      return;
    }
    fsOut << "Vector_of_motionTrack" << "[";
    for( unsigned int i=0;i<vec_points_to_track.size( ); i++ )
    {
      PointsToTrack::write( fsOut,*vec_points_to_track[ i ] );
    }
    fsOut << "]";
    fsOut.release( );

  }
  cout<<"Create the sequence analyzer:"<<endl;

  SequenceAnalyzer motion_estim( vec_points_to_track, &images, matches_algo );

  pathFileTracks = FROM_SRC_ROOT( "Medias/tracks_points_"POINT_METHOD"/motion_tracks1.yml" );
  cout<<"Compute matches between each frames..."<<endl<<endl;
  cout<<"The complexity is O( scary ) (approx n^2), so be patient...\nOnce done "
    "you will not have to do it again, everything will be saved in:\n"<<
    pathFileTracks<<"..."<<endl;
  motion_estim.computeMatches( );

  vector<TrackOfPoints> &tracks=motion_estim.getTracks( );
  cout<<"numbers of tracks:"<<tracks.size( )<<endl;

  //now save the tracks:
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

  cout<<"We will now remove bad points matches..."<<endl;
  //int min_point_sequence = MAX( images.size()/30, 2 );
  SequenceAnalyzer::keepOnlyCorrectMatches(motion_estim, 2, 0 );

  tracks=motion_estim.getTracks( );
  cout<<"numbers of correct tracks:"<<tracks.size( )<<endl;

  //now save the tracks:
  pathFileTracks = FROM_SRC_ROOT( "Medias/tracks_points_"POINT_METHOD"/motion_tracks.yml" );
  FileStorage fsOutMotion( pathFileTracks, FileStorage::WRITE );
  if( !fsOutMotion.isOpened( ) )
  {
    cout<<"Can't create "<<pathFileTracks<<"!\nPlease verify you have access to the directory!"<<endl;
    return;
  }
  //Can't find a way to enable the following notation:
  //fs << *ptt1;
  SequenceAnalyzer::write( fsOutMotion,motion_estim );
  fsOutMotion.release( );
  //now for fun show the sequence on images:
  motion_estim.showTracks( 0 );


  /*
  //and create a new PointsToTrack using this file:
  vector<Ptr<PointsToTrack>> points_empty;
  MotionEstimator motion_estim_loaded( points_empty, matches_algo->clone( true ) );
  //ptt_New=Ptr<PointsToTrack>( new PointsToTrack ( ) );
  FileStorage fsRead( "motion_tracks.yml", FileStorage::READ );
  FileNode myPtt = fsRead.getFirstTopLevelNode( );
  //Can't find a way to enable the following notation:
  //myPtt >> ptt_New;
  MotionEstimator::read( myPtt, motion_estim_loaded );
  fsRead.release( );

  vector<TrackPoints> &tracks=motion_estim_loaded.getTracks( );
  cout<<"numbers of correct tracks:"<<tracks.size( )<<endl;
  motion_estim_loaded.showTracks( images,0 );*/
}
