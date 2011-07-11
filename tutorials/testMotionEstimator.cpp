
#include "../src/PointsToTrackWithImage.h"
#include "../src/MotionProcessor.h"
#include "../src/SequenceAnalyzer.h"
#include "../src/PointsMatcher.h"
#include "../src/libmv_mapping.h"
#include "config.h"
#include <opencv2/calib3d/calib3d.hpp>

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

#include "test_data_sets.h"

NEW_TUTO(Track_creation, "Learn how you can compute tracks from a list of pictures",
  "Using features detector and matcher, create a vector of tracks (a track is a list of 2D points from the same 3D point).\nYou will also learn how you can save an object using YAML!")
{
  MotionProcessor mp;
  //first load images:
  //Here we will a folder with a lot of images, but we can do the same thing with any other type of input
  if( !mp.setInputSource(FROM_SRC_ROOT("Medias/temple/"),IS_DIRECTORY) )
  {
    cout<<"Can't find \"Medias/temple/\" dataset"<<endl;
    return;
  }

  //Configure input (not needed, but show how we can do 
  //mp.setProperty(CV_CAP_PROP_CONVERT_RGB,0);//Only greyscale, due to SIFT

  Ptr<FeatureDetector> detector;
  detector=Ptr<FeatureDetector>(new FastFeatureDetector());
  Ptr<DescriptorExtractor> desc_extractor;
  desc_extractor=Ptr<DescriptorExtractor>(new BriefDescriptorExtractor(32));
  vector<Ptr<PointsToTrack>> vec_points_to_track;
  Ptr<PointsToTrack> ptrPoints_tmp;

  Ptr<DescriptorMatcher> matcher;
  matcher=Ptr<DescriptorMatcher>(new BruteForceMatcher<Hamming>());
  Ptr<PointsMatcher> matches_algo ( new PointsMatcher(matcher) );

  //universal method to get the current image:
  vector<Mat> images;
  Mat currentImage=mp.getFrame();

  string pathFileTracks = FROM_SRC_ROOT("Medias/tracks_points_BRIEF/motion_points.yml");
  std::ifstream inPoints(pathFileTracks.c_str());
  if( inPoints.is_open() )
  {
    inPoints.close();
    cout<<"Load points from \"motion_points.yml\""<<endl;
    FileStorage fsRead(pathFileTracks, FileStorage::READ);
    FileNodeIterator vecPtt = fsRead.getFirstTopLevelNode().begin();
    FileNodeIterator vecPtt_end = fsRead.getFirstTopLevelNode().end();

    while( vecPtt != vecPtt_end && !currentImage.empty() )
    {
      Ptr<PointsToTrack> ptt = Ptr<PointsToTrack>(new PointsToTrack());
      PointsToTrack::read( (*vecPtt)[0]["PointsToTrack"],*ptt);
      vec_points_to_track.push_back(ptt);
      images.push_back(currentImage);
      currentImage=mp.getFrame();
      vecPtt++;
      cout<<"|";
    }
    cout<<endl;

    fsRead.release();
  }
  else
  {
    int nbFrame=0;
    while ( !currentImage.empty() )// && nbFrame<50 )
    {
      //if the image is loaded, find the points:
      cout<<"Create a new PointsToTrack..."<<endl;

      ptrPoints_tmp = Ptr<PointsToTrack>( new PointsToTrackWithImage (nbFrame,
        currentImage, Mat(), detector, desc_extractor));
      ptrPoints_tmp->computeKeypointsAndDesc();
      
      vec_points_to_track.push_back( ptrPoints_tmp );
      images.push_back(currentImage);
      nbFrame++;
      currentImage=mp.getFrame();
    }

    //now save the tracks:
    FileStorage fsOut(pathFileTracks, FileStorage::WRITE);
    if( !fsOut.isOpened() )
    {
      cout<<"Can't create "<<pathFileTracks<<"!\nPlease verify you have access to the directory!"<<endl;
      return;
    }
    fsOut << "Vector_of_motionTrack" << "[";
    for(unsigned int i=0;i<vec_points_to_track.size(); i++)
    {
      PointsToTrack::write(fsOut,*vec_points_to_track[i]);
    }
    fsOut << "]";
    fsOut.release();

  }
  cout<<"Create the sequence analyzer:"<<endl;

  SequenceAnalyzer motion_estim(vec_points_to_track,matches_algo,images);

  motion_estim.computeMatches();

  vector<TrackPoints> &tracks=motion_estim.getTracks();
  cout<<"numbers of tracks:"<<tracks.size()<<endl;

  //now save the tracks:
  pathFileTracks = FROM_SRC_ROOT("Medias/tracks_points_BRIEF/motion_tracks1.yml");
  FileStorage fsOutMotion1(pathFileTracks, FileStorage::WRITE);
  if( !fsOutMotion1.isOpened() )
  {
    cout<<"Can't create "<<pathFileTracks<<"!\nPlease verify you have access to the directory!"<<endl;
    return;
  }
  //Can't find a way to enable the following notation:
  //fs << *ptt1;
  SequenceAnalyzer::write(fsOutMotion1,motion_estim);
  fsOutMotion1.release();

  motion_estim.keepOnlyCorrectMatches();

  tracks=motion_estim.getTracks();
  cout<<"numbers of correct tracks:"<<tracks.size()<<endl;

  //now save the tracks:
  pathFileTracks = FROM_SRC_ROOT("Medias/tracks_points_BRIEF/motion_tracks.yml");
  FileStorage fsOutMotion(pathFileTracks, FileStorage::WRITE);
  if( !fsOutMotion.isOpened() )
  {
    cout<<"Can't create "<<pathFileTracks<<"!\nPlease verify you have access to the directory!"<<endl;
    return;
  }
  //Can't find a way to enable the following notation:
  //fs << *ptt1;
  SequenceAnalyzer::write(fsOutMotion,motion_estim);
  fsOutMotion.release();
  //now for fun show the sequence on images:
  motion_estim.showTracks(0);

  
  /*
  //and create a new PointsToTrack using this file:
  vector<Ptr<PointsToTrack>> points_empty;
  MotionEstimator motion_estim_loaded( points_empty, matches_algo->clone(true) );
  //ptt_New=Ptr<PointsToTrack>(new PointsToTrack ());
  FileStorage fsRead("motion_tracks.yml", FileStorage::READ);
  FileNode myPtt = fsRead.getFirstTopLevelNode();
  //Can't find a way to enable the following notation:
  //myPtt >> ptt_New;
  MotionEstimator::read(myPtt, motion_estim_loaded);
  fsRead.release();

  vector<TrackPoints> &tracks=motion_estim_loaded.getTracks();
  cout<<"numbers of correct tracks:"<<tracks.size()<<endl;
  motion_estim_loaded.showTracks(images,0);*/
}
