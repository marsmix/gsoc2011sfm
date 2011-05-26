#include "PointsToTrack.h"

using cv::Mat;
using cv::Scalar;
using cv::KeyPoint;
using cv::Point;
using std::vector;
using cv::line;
using cv::circle;

namespace OpencvSfM{
  PointsToTrack::PointsToTrack(std::vector<KeyPoint> keypoints/*=std::vector<cv::KeyPoint>(0)*/,Mat descriptors/*=cv::Mat()*/)
  {
  }


  PointsToTrack::~PointsToTrack(void)
  {
    keypoints_.clear();
    descriptors_.release();
  }

  void PointsToTrack::printPointsOnImage(Mat &image,bool withSize,Scalar color,int thickness) const
  {
    vector<KeyPoint>::const_iterator iterOnPoints=keypoints_.begin();
    while(iterOnPoints!=keypoints_.end()){
      if(withSize)
      {
        float sizeOfPoint=iterOnPoints->size;
        if(iterOnPoints->size<1.0)
          sizeOfPoint=1;//because the circle won't be correctly drawn!
        if(iterOnPoints->angle!=-1)//the angle information is set!
        {                          //Draw the orientation:
          double sin_dir = sin((float)iterOnPoints->angle*(CV_PI/180.0));
          double cos_dir = cos((float)iterOnPoints->angle*(CV_PI/180.0));

          line(image, Point((int)iterOnPoints->pt.x,(int)iterOnPoints->pt.y),
            Point((int)(iterOnPoints->pt.x+sizeOfPoint*cos_dir),(int)(iterOnPoints->pt.y-sizeOfPoint*sin_dir)),color,thickness);
        }
        circle( image, Point((int)iterOnPoints->pt.x,(int)iterOnPoints->pt.y),(int)sizeOfPoint,color,thickness);
      }
      else
      {
        circle( image, Point((int)iterOnPoints->pt.x,(int)iterOnPoints->pt.y),(int)thickness,color,-thickness);
      }
      iterOnPoints++;
    }
  }
}