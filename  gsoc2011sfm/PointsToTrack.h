#ifndef _GSOC_SFM_POINTS_TO_TRACK_H
#define _GSOC_SFM_POINTS_TO_TRACK_H 1

#include "opencv2/features2d/features2d.hpp"

#include <vector>


namespace OpencvSfM{

/*! \brief This class can be used to store informations about points and features. 
  *      This is an abstract class: you can't use it directly. Use for instance PointsToTrackSIFT.
  *
  * To create a structure from motion, most methods use points to compute the structure.
  * This class focus on the first task in SfM: find points in image which are easy to track...
  * When available, a feature vector for each points is very helpful: the matching will be easier.
  */
  class PointsToTrack
  {
  protected:
    std::vector<cv::KeyPoint> keypoints_;///<This attribute will store points coordinates and sometimes orientation and size
    cv::Mat descriptors_;///<this attribute will store descritors for each points in a matrix with size (n*m), where n is the number of points and m is the desciptor size.
  public:
    /**
    * this constructor create an object with available information...
    * @param keypoints the points we will try to track...
    * @param descriptors the feature vector for each points...
    */
    PointsToTrack(std::vector<cv::KeyPoint> keypoints=std::vector<cv::KeyPoint>(0),cv::Mat descriptors=cv::Mat());
    /**
    * Destructor : delete points and features vectors
    */
    virtual ~PointsToTrack(void);
    
    /**
    * This method is used to compute both Keypoints and descriptors...
    * @return the number of points
    */
    virtual int computeKeypointsAndDesc()=0;
    /**
    * This method is used to compute only Keypoints...
    * @return the number of points
    */
    virtual int computeKeypoints()=0;
    /**
    * This method is used to compute only descriptors...
    * the keypoints given in parameters a mixed with previous computed keypoints (if they exist) and features are computed
    * @param keypoints points are added to keypoints_ before descriptor computation.
    */
    virtual void computeDescriptors(std::vector<cv::KeyPoint> keypoints=std::vector<cv::KeyPoint>(0))=0;
    /**
    * To show the points on image, use this function to draw points on it.
    * @param image surface where points will be draw
    * @param withSize if true, points are drawn using circle with radius=size, else only points.
    * @param color color to use when drawing the points
    * @param thickness – Thickness of the points outline
    */
    void printPointsOnImage(cv::Mat &image,bool withSize=true,cv::Scalar color=cv::Scalar(0),int thickness = 1) const;
  };

}

#endif