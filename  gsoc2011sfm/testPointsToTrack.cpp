#include "PointsToTrackWithImage.h"
#include "MotionProcessor.h"

#include <iostream>

using namespace std;
using namespace cv;
using namespace OpencvSfM;

//////////////////////////////////////////////////////////////////////////
//Only to see how we can use PointsToTracks
//This file will not be in the final version of API, consider it like a tuto/draft...
//You will need files to test. Download the temple dataset here : http://vision.middlebury.edu/mview/data/
//////////////////////////////////////////////////////////////////////////

void main(){
  MotionProcessor mp;

  //first load image:
  //Here we will only open an image, but we can do the same thing with any other type of input
  //The goal below this class is to open a folder of images the same way we open a video stream.
  //For example, we could set input to webcam like this :
  //mp.setInputSource(0);
  mp.setInputSource("../Medias/temple/temple0001.png");

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
    Ptr<DescriptorExtractor> SIFTDetect;
    SIFTDetect=Ptr<DescriptorExtractor>(new SiftDescriptorExtractor());

    PointsToTrackWithImage ptt(imgT0001,Mat(),fastDetect,SIFTDetect);
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
