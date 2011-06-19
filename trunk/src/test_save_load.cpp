
//Set to 1 if you want to test the points detection and matching
//But be aware to set other tests to 0...
#if 0

#include "PointsToTrackWithImage.h"
#include "MotionProcessor.h"
#include "MotionEstimator.h"
#include "PointsMatcher.h"
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
//Only to see how we can use PointsMatcher
//////////////////////////////////////////////////////////////////////////

void main(){

  MotionProcessor mp;
  vector<Mat> masks;

  //first load images:
  //Here we will a folder with a lot of images, but we can do the same thing with any other type of input
  mp.setInputSource("../Medias/temple/",IS_DIRECTORY);

  //Configure input (not needed, but show how we can do 
  mp.setProperty(CV_CAP_PROP_CONVERT_RGB,0);//Only greyscale, due to SIFT
  mp.setProperty(CV_CAP_PROP_FRAME_WIDTH,1024);//for test
  mp.setProperty(CV_CAP_PROP_FRAME_HEIGHT,768);//idem...

  //universal method to get the current image:
  Mat firstImage=mp.getFrame();

  if(firstImage.empty())
  {
    cout<<"test can not be run... can't find images..."<<endl;
  }
  else
  {
    //if the images are loaded, find the points:
    cout<<"creation of two detection algorithm..."<<endl;
    Ptr<FeatureDetector> fastDetect;
    fastDetect=Ptr<FeatureDetector>(new SurfFeatureDetector());
    Ptr<DescriptorExtractor> SurfDetect;
    SurfDetect=Ptr<DescriptorExtractor>(new SurfDescriptorExtractor());

    cout<<"now create the two set of points with features..."<<endl;
    Ptr<PointsToTrack> ptt1;
    ptt1=Ptr<PointsToTrack>(new PointsToTrackWithImage (
      firstImage,Mat(),fastDetect,SurfDetect));
    ptt1->computeKeypointsAndDesc(true);

    Ptr<DescriptorMatcher> matcher;
    matcher=Ptr<DescriptorMatcher>(new FlannBasedMatcher());
    Ptr<PointsMatcher> matches_algo ( new PointsMatcher(matcher) );

    matches_algo->add(ptt1);

    //now save the points:
    FileStorage fsOut("test.yml", FileStorage::WRITE);
    //Can't find a way to enable the following notation:
    //fs << *ptt1;
    PointsMatcher::write(fsOut,*matches_algo);
    fsOut.release();

    //and create a new PointsToTrack using this file:
    Ptr<PointsMatcher> matches_new ( new PointsMatcher(matcher->clone(true)) );
    //ptt_New=Ptr<PointsToTrack>(new PointsToTrack ());
    FileStorage fsRead("test.yml", FileStorage::READ);
    FileNode myPtt = fsRead.getFirstTopLevelNode();
    //Can't find a way to enable the following notation:
    //myPtt >> ptt_New;
    PointsMatcher::read(myPtt, *matches_new);
    fsRead.release();
  }

}

#endif