
#include "../src/PointsToTrackWithImage.h"
#include "../src/MotionProcessor.h"
#include "../src/SequenceAnalyzer.h"
#include "../src/PointsMatcher.h"
#include <opencv2/calib3d/calib3d.hpp>

//////////////////////////////////////////////////////////////////////////
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

#include "test_data_sets.h"

NEW_TUTO(Points_Definitions, "How features can be defined",
  "Using 2 pictures loaded using a motion processor, we find points, match them and find the fundamental matrix."){

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
  Mat secondImage=mp.getFrame();

  if(firstImage.empty()||secondImage.empty())
  {
    cout<<"test can not be run... can't find different images..."<<endl;
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
    ptt1=Ptr<PointsToTrack>(new PointsToTrackWithImage (0, firstImage,Mat(),fastDetect,SurfDetect));
    Ptr<PointsToTrack> ptt2;
    ptt2=Ptr<PointsToTrack>(new PointsToTrackWithImage (1, secondImage,Mat(),fastDetect,SurfDetect));

    cout<<"now try to find matches, so we create a matcher (classic bruteForce)"<<endl<<endl;
    Ptr<DescriptorMatcher> matcher;
    matcher=Ptr<DescriptorMatcher>(new FlannBasedMatcher());

    //The matches vector is:
    vector<DMatch> matchesVector;
    
    //The point matcher will now be created like this:
    PointsMatcher matches(matcher);

    //We want to find points in img1 which are also in img2.
    //So we set ptt2 as training data:
    cout<<"Add points of image 2 as references points"<<endl;
    matches.add(ptt2);

    Ptr<PointsMatcher> matches2= matches.clone();
    matches2->add(ptt1);
    //cross check matches:
    matches.crossMatch(matches2,matchesVector);

    Mat outImg;
    cout<<"Displaying the "<<matchesVector.size()<<"points..."<<endl;
    drawMatches(firstImage, ptt1->getKeypoints(), secondImage,
      ptt2->getKeypoints(), matchesVector, outImg);
    imshow("PointsMatcher key points",outImg);
    cv::waitKey(0);

    //////////////////////////////////////////////////////////////////////////
    //For now we use fundamental function from OpenCV but soon use libmv !

    //First compute points matches:
    int size_match=matchesVector.size();
    Mat srcP(1,size_match,CV_32FC2);
    Mat destP(1,size_match,CV_32FC2);
    vector<uchar> status;

    //vector<KeyPoint> points1 = point_matcher->;
    for( int i = 0; i < size_match; i ++ ){
      const KeyPoint &key1 = ptt1->getKeypoint(
        matchesVector[i].queryIdx);
      const KeyPoint &key2 = ptt2->getKeypoint(
        matchesVector[i].trainIdx);
      srcP.at<float[2]>(0,i)[0] = key1.pt.x;
      srcP.at<float[2]>(0,i)[1] = key1.pt.y;
      destP.at<float[2]>(0,i)[0] = key2.pt.x;
      destP.at<float[2]>(0,i)[1] = key2.pt.y;
      status.push_back(1);
    }

    Mat fundam = cv::findFundamentalMat(srcP, destP, status, cv::FM_RANSAC);

    //refine the mathing :
    for( int i = 0; i < size_match; ++i ){
      if( status[i] == 0 )
      {
        status[i] = status[--size_match];
        status.pop_back();
        matchesVector[i--] = matchesVector[size_match];
        matchesVector.pop_back();
      }
    }

    Mat outImg1;
    cout<<"Displaying the "<<matchesVector.size()<<"points..."<<endl;
    drawMatches(firstImage, ptt1->getKeypoints(), secondImage,
      ptt2->getKeypoints(), matchesVector, outImg1);
    imshow("PointsMatcher key points1",outImg1);
    cv::waitKey(0);
  }
}


//////////////////////////////////////////////////////////////////////////
//Only to see how we can use PointsToTracks
//////////////////////////////////////////////////////////////////////////
/*
void main(){
  MotionProcessor mp;

  //first load image:
  //Here we will only open an image, but we can do the same thing with any other type of input
  //The goal below this class is to open a folder of images the same way we open a video stream.
  //For example, we could set input to webcam like this :
  //mp.setInputSource(0);
  mp.setInputSource("../Medias/temple/temple0001.png",IS_SINGLE_FILE);

  //Configure input (not needed, but show how we can do 
  mp.setProperty(CV_CAP_PROP_CONVERT_RGB,0);//Only greyscale, due to SIFT
  mp.setProperty(CV_CAP_PROP_FRAME_WIDTH,600);//for test
  mp.setProperty(CV_CAP_PROP_FRAME_HEIGHT,450);//idem...

  //universal method to get the current image:
  Mat imgWithPoints;
  Mat imgT0001=mp.getFrame();
  if(imgT0001.empty())
  {
    cout<<"test can not be run... temple0001.png is not found..."<<endl;
  }
  else
  {
    //if the image is loaded, find the points:
    int threshold=5;

    //create the two detection algorithm:
    Ptr<FeatureDetector> fastDetect;
    fastDetect=Ptr<FeatureDetector>(new FastFeatureDetector(threshold));
    Ptr<DescriptorExtractor> SurfDetect;
    SurfDetect=Ptr<DescriptorExtractor>(new SiftDescriptorExtractor());

    PointsToTrackWithImage ptt(imgT0001,Mat(),fastDetect,SurfDetect);
    int nbPoints;
    for(int i=0;i<10;i++){
      nbPoints=ptt.computeKeypointsAndDesc();
      cout<<nbPoints<<" points found using "<<threshold<<endl;
      //show key points:
      //print points on it:
      ptt.printPointsOnImage(imgT0001,imgWithPoints,Scalar(255));
      imshow("PointsToTrackWithImage key points",imgWithPoints);
      cv::waitKey(20);

      threshold++;
      ptt.setFeatureDetector(Ptr<FeatureDetector>(new FastFeatureDetector(threshold)));
    }
  }
}
*/