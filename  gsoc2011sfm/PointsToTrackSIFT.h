#ifndef _GSOC_SFM_POINTS_TO_TRACK_SIFT_H
#define _GSOC_SFM_POINTS_TO_TRACK_SIFT_H 1

#include "pointstotrack.h"

namespace OpencvSfM{

/*! \brief This class can be used to find points and features in pictures using SIFT detector
  *
  * To create a structure from motion, most methods use points to compute the structure.
  * This class focus on the first task in SfM: find points in image which are easy to track...
  * When available, a feature vector for each points is very helpful: the matching will be easier.
  */
  class PointsToTrackSIFT :
    public PointsToTrack
  {
  protected:
    cv::SIFT sift_detector_;///<SIFT class which will find the points
    cv::Mat imageToAnalyse_;///<Picture from where points are detected
    cv::Mat maskOfAnalyse_;///<Mask of analyse. Everything out of this mask is ignored.
  public:
    PointsToTrackSIFT(cv::Mat imageToAnalyse,cv::Mat maskOfAnalyse,double threshold, double edgeThreshold,
      int nOctaves=cv::SIFT::CommonParams::DEFAULT_NOCTAVES,
      int nOctaveLayers=cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
      int firstOctave=cv::SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
      int angleMode=cv::SIFT::CommonParams::FIRST_ANGLE);

    PointsToTrackSIFT(cv::Mat imageToAnalyse,cv::Mat maskOfAnalyse,double _magnification=cv::SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(),
      bool _isNormalize=true,
      bool _recalculateAngles = true,
      int _nOctaves=cv::SIFT::CommonParams::DEFAULT_NOCTAVES,
      int _nOctaveLayers=cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
      int _firstOctave=cv::SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
      int _angleMode=cv::SIFT::CommonParams::FIRST_ANGLE );

    PointsToTrackSIFT(cv::Mat imageToAnalyse,cv::Mat maskOfAnalyse,const cv::SIFT::CommonParams& _commParams,
      const cv::SIFT::DetectorParams& _detectorParams = cv::SIFT::DetectorParams(),
      const cv::SIFT::DescriptorParams& _descriptorParams = cv::SIFT::DescriptorParams());
    ~PointsToTrackSIFT(void);

    /**
    * This method is used to compute both Keypoints and descriptors...
    * @return the number of points
    */
    int computeKeypointsAndDesc();
    /**
    * This method is used to compute only Keypoints...
    * @return the number of points
    */
    int computeKeypoints();
    /**
    * This method is used to compute only descriptors...
    * the keypoints given in parameters a mixed with previous computed keypoints (if they exist) and features are computed
    * @param keypoints points are added to keypoints_ before descriptor computation.
    */
    void computeDescriptors(std::vector<cv::KeyPoint> keypoints=std::vector<cv::KeyPoint>(0));

  };

}

#endif