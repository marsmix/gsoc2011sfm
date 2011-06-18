#ifndef _GSOC_SFM_POINTS_TO_TRACK_H
#define _GSOC_SFM_POINTS_TO_TRACK_H 1

#include "opencv2/features2d/features2d.hpp"

#include <vector>


namespace OpencvSfM{

/*! \brief This class can be used to store informations about point
  * and features. This is an abstract class: you can't use it directly.
  * Use for instance PointsToTrackWithImage.
  *
  * To create a structure from motion, most methods use points to compute
  * the structure. This class focus on the first task in SfM: find points
  * in image which are easy to track...
  * When available, a feature vector for each points is very helpful:
  * the matching will be easier.
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
    * @param forcing_recalculation if true previous keypoints are removed...
    * If false and if keypoints and descriptor exists, nothing is done.
    * @return the number of points
    */
    virtual int computeKeypointsAndDesc(bool forcing_recalculation=false);
    /**
    * This method is used to compute only Keypoints...
    * @return the number of points
    */
    virtual int computeKeypoints();
    /**
    * This method is used to compute only descriptors...
    */
    virtual void computeDescriptors();
    /**
    * This method is used to add Keypoints...
    * @param keypoints Keypoints to add
    * @param descriptors of points, if any
    * @param computeMissingDescriptor if true, the missing descriptors are computed.
    */
    virtual void addKeypoints(std::vector<cv::KeyPoint> keypoints,cv::Mat descriptors=cv::Mat(),bool computeMissingDescriptor=false);
    /**
    * this method return the points coordinates and sometimes orientation and size
    * @return points coordinates and sometimes orientation and size
    */
    std::vector<cv::KeyPoint>& getKeypoints() {return keypoints_;};
    /**
    * this method return the descritors for each points in a matrix with size (n*m), where n is the number of points and m is the desciptor size.
    * @return descritors for each points in a matrix with size (n*m), where n is the number of points and m is the desciptor size.
    */
    cv::Mat getDescriptors() const {return descriptors_;};
    /**
    * To show the points on image, use this function to draw points on it.
    * @param image Source image.
    * @param outImg Output image. Its content depends on flags value what is drawn in output image. See possible flags bit values.
    * @param color Color of keypoints
    * @param flags Possible flags bit values is defined by DrawMatchesFlags (see http://opencv.willowgarage.com/documentation/cpp/features2d_drawing_function_of_keypoints_and_matches.html#cv-drawmatches)
    */
    //
    void printPointsOnImage(const cv::Mat &image, cv::Mat& outImg, const cv::Scalar& color=cv::Scalar::all(-1), int flags=cv::DrawMatchesFlags::DEFAULT) const;

    static void read( const cv::FileNode& node, PointsToTrack& points );

    static void write ( cv::FileStorage& fs, const PointsToTrack& points );
  };
}

//! writes vector of keypoints to the file storage
void cv::write( cv::FileStorage& fs, const std::string& name,
  const OpencvSfM::PointsToTrack& points )
{
  OpencvSfM::PointsToTrack::write( fs,points );
};
//! reads vector of keypoints from the specified file storage node
void cv::read(const cv::FileNode& node, OpencvSfM::PointsToTrack& points )
{
  OpencvSfM::PointsToTrack::read( node,points );
}

#endif