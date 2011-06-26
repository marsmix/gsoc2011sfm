//Set to 1 if you want to test the points detection and matching
//But be aware to set other tests to 0...
#if 0

#include "PointsToTrackWithImage.h"
#include "MotionProcessor.h"
#include "MotionEstimator.h"
#include "PointsMatcher.h"
#include "libmv_mapping.h"
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

using namespace std;
using namespace cv;
using namespace OpencvSfM;

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//Only to see how we can use MotionEstimator
//////////////////////////////////////////////////////////////////////////

void main(){
  MotionProcessor mp;

  //first load images:
  //Here we will a folder with a lot of images, but we can do the same thing with any other type of input
  mp.setInputSource("../Medias/temple/",IS_DIRECTORY);

  //Configure input (not needed, but show how we can do 
  //mp.setProperty(CV_CAP_PROP_CONVERT_RGB,0);//Only greyscale, due to SIFT

  Ptr<FeatureDetector> detector;
  detector=Ptr<FeatureDetector>(new SiftFeatureDetector());
  Ptr<DescriptorExtractor> desc_extractor;
  desc_extractor=Ptr<DescriptorExtractor>(new SiftDescriptorExtractor());
  vector<Ptr<PointsToTrack>> vec_points_to_track;
  Ptr<PointsToTrack> ptrPoints_tmp;

  Ptr<DescriptorMatcher> matcher;
  matcher=Ptr<DescriptorMatcher>(new FlannBasedMatcher());
  Ptr<PointsMatcher> matches_algo ( new PointsMatcher(matcher) );

  //universal method to get the current image:
  vector<Mat> images;
  Mat currentImage=mp.getFrame();

  if( boost::filesystem::exists( boost::filesystem::status("motion_points.yml") ) )
  {
    cout<<"Load points from \"motion_points.yml\""<<endl;
    FileStorage fsRead("motion_points.yml", FileStorage::READ);
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

      ptrPoints_tmp = Ptr<PointsToTrack>( new PointsToTrackWithImage (
        currentImage, Mat(), detector, desc_extractor));
      ptrPoints_tmp->computeKeypointsAndDesc();
      
      vec_points_to_track.push_back( ptrPoints_tmp );
      images.push_back(currentImage);
      nbFrame++;
      currentImage=mp.getFrame();
    }
    cout<<"Create the motion estimator:"<<endl;

    //now save the tracks:
    FileStorage fsOut("motion_points.yml", FileStorage::WRITE);
    fsOut << "Vector_of_motionTrack" << "[";
    for(int i=0;i<vec_points_to_track.size(); i++)
    {
      PointsToTrack::write(fsOut,*vec_points_to_track[i]);
    }
    fsOut << "]";
    fsOut.release();

  }

  MotionEstimator motion_estim(vec_points_to_track,matches_algo);

  motion_estim.computeMatches();

  vector<TrackPoints> &tracks=motion_estim.getTracks();
  cout<<"numbers of tracks:"<<tracks.size()<<endl;

  //now save the tracks:
  FileStorage fsOutMotion1("motion_tracks1.yml", FileStorage::WRITE);
  //Can't find a way to enable the following notation:
  //fs << *ptt1;
  MotionEstimator::write(fsOutMotion1,motion_estim);
  fsOutMotion1.release();

  motion_estim.keepOnlyCorrectMatches();

  tracks=motion_estim.getTracks();
  cout<<"numbers of correct tracks:"<<tracks.size()<<endl;

  //now for fun show the sequence on images:
  motion_estim.showTracks(images,1000);

  //now save the tracks:
  FileStorage fsOutMotion("motion_tracks.yml", FileStorage::WRITE);
  //Can't find a way to enable the following notation:
  //fs << *ptt1;
  MotionEstimator::write(fsOutMotion,motion_estim);
  fsOutMotion.release();
  
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


#endif