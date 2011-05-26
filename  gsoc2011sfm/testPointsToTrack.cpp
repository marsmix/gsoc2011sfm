#include "PointsToTrackSIFT.h"
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
  Mat imgT0001=mp.getFrame();
  if(imgT0001.empty())
  {
    cout<<"test can not be run... temple0001.png is not found..."<<endl;
  }
  else
  {
    //if the image is loaded, find the points:
    double threshold=SIFT::DetectorParams::GET_DEFAULT_THRESHOLD()/2;
    PointsToTrackSIFT ptt(imgT0001,Mat(),threshold,SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION());
    int nbPoints;
    Mat imgTmp;
    for(int i=0;i<10;i++){
      nbPoints=ptt.computeKeypointsAndDesc();
      cout<<nbPoints<<" points found using "<<threshold<<" as SIFT threshold and computeKeypointsAndDesc"<<endl;
      //show key points:
      imgTmp=imgT0001.clone();
      //print points on it:
      ptt.printPointsOnImage(imgTmp,false,Scalar(255));
      imshow("PointsToTrackSIFT key points",imgTmp);


      nbPoints=ptt.computeKeypoints();
      cout<<nbPoints<<" points found using "<<threshold<<" as SIFT threshold and computeKeypoints"<<endl;
      //show key points:
      imgTmp=imgT0001.clone();
      //print points on it:
      ptt.printPointsOnImage(imgTmp,false,Scalar(255));
      imshow("PointsToTrackSIFT key points",imgTmp);


      threshold*=1.1;
      ptt=PointsToTrackSIFT(imgT0001,Mat(),threshold,SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION());
    }
  }
}
