//Set to 1 if you want to test the points detection and matching
//But be aware to set other tests to 0...
#if 0

#include "../src/SequenceAnalyzer.h"
#include "../src/PointsMatcher.h"
#include "../src/PointsToTrackWithImage.h"
#include "../src/MotionProcessor.h"
#include "../src/PointOfView.h"
#include "../src/CameraPinhole.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>

#include <iostream>

using namespace std;
using namespace cv;
using namespace OpencvSfM;

void main(){

  int nviews = 5;
  int npoints = 6;
  std::ifstream inPoints("logPoints.txt");
  string skeepWord;

  // Collect P matrices together.
  vector<PointOfView> cameras;
  for (int j = 0; j < nviews; ++j) {
    double R[9];
    double T[3];
    double K[9];
    inPoints >> skeepWord;
    while( skeepWord.find("ProjectionsMat")==std::string::npos && !inPoints.eof() )
      inPoints >> skeepWord;
    inPoints >> K[0] >> K[1] >> K[2];
    inPoints >> K[3] >> K[4] >> K[5];
    inPoints >> K[6] >> K[7] >> K[8];
    Ptr<Camera> device=Ptr<Camera>(new CameraPinhole(Mat(3,3,CV_64F,K)) );
    inPoints >> R[0] >> R[1] >> R[2];
    inPoints >> R[3] >> R[4] >> R[5];
    inPoints >> R[6] >> R[7] >> R[8];
    inPoints >> T[0] >> T[1] >> T[2];
    //create the PointOfView:
    cameras.push_back( PointOfView(device, Mat(3,3,CV_64F,R), Vec3d(T) ) );
  }

  vector<cv::Vec3d> points3D;
  vector<cv::Ptr<PointsToTrack>> points2D;
  vector<vector<cv::KeyPoint>> tracks_keypoints(nviews);
  for (int i = 0; i < npoints; ++i) {
    inPoints >> skeepWord;
    while( skeepWord != "2D" && !inPoints.eof() )
      inPoints >> skeepWord;
    inPoints >> skeepWord;//skip Point
    double X[3];
    inPoints >> X[0] >> X[1] >> X[2];
    points3D.push_back( Vec3d( X[0], X[1], X[2] ) );

    // Collect the image of point i in each frame.
    // inPoints >> skeepWord;
    for (int j = 0; j < nviews; ++j) {
      cv::Vec2d point;
      inPoints >> point[0] >> point[1];
      tracks_keypoints[j].push_back( KeyPoint(cv::Point2f(point[0], point[1]),1) );
    }
  }
  for (int j = 0; j < nviews; ++j) {
    points2D.push_back( cv::Ptr<PointsToTrack>(
      new PointsToTrack( tracks_keypoints[j] )) );
  }
  /*
  vector<Mat> Ks(nviews);
  vector<Mat> Ps(nviews);
  vector<cv::Vec3d> points3D;
  vector<vector<cv::Vec2d>> points2D;
  */

  Ptr<DescriptorMatcher> matcher;
  matcher=Ptr<DescriptorMatcher>(new FlannBasedMatcher());
  Ptr<PointsMatcher> matches_algo ( new PointsMatcher(matcher) );
  MotionEstimator motion_estim(points2D,matches_algo);

  //create tracks:
  vector<TrackPoints> tracks;
  for(int i=0; i<npoints; ++i)
  {
    TrackPoints tp;
    for(int j=0;j<nviews;j++)
    {
      tp.addMatch(j,i);
    }
    tracks.push_back(tp);
  }
  motion_estim.addTracks(tracks);

  //create an empty sequence of images:
  vector<Mat> images;
  for (int j = 0; j < nviews; ++j) {
    Mat img=Mat::zeros(800,600,CV_8SC1);
    images.push_back( img );
  }

  //now for fun show the sequence on images:
  motion_estim.triangulateNViewDebug(cameras,images,points3D);
}

#endif