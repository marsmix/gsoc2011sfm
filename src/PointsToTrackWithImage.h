#ifndef _GSOC_SFM_POINTS_TO_TRACK_SIFT_H
#define _GSOC_SFM_POINTS_TO_TRACK_SIFT_H 1

#include "macro.h" //SFM_EXPORTS
#include "PointsToTrack.h"

namespace OpencvSfM{

/*! \brief This class can be used to find points and features in pictures using SIFT detector
  *
  * To create a structure from motion, most methods use points to compute the structure.
  * This class focus on the first task in SfM: find points in image which are easy to track...
  * When available, a feature vector for each points is very helpful: the matching will be easier.
  */
  class SFM_EXPORTS PointsToTrackWithImage :
    public PointsToTrack
  {
  protected:
    cv::Ptr<cv::FeatureDetector> feature_detector_;///<class which will find the points
    cv::Ptr<cv::DescriptorExtractor> descriptor_detector_;///<class which will compute the descriptors
    cv::Mat imageToAnalyse_;///<Picture from where points are detected
    cv::Mat maskOfAnalyse_;///<Mask of analyse. Everything out of this mask is ignored.
    
    /**
    * This method is used to compute only Keypoints...
    * @return the number of points
    */
    int impl_computeKeypoints_( );
    /**
    * This method is used to compute only descriptors...
    */
    void impl_computeDescriptors_( );
  public:
    /**
    * First constructor used to create a list of points to track using a feature and a descriptor algorithm.
    * @param corresponding_image Global index of image
    * @param imageToAnalyse Image to use for keypoints and features search
    * @param maskOfAnalyse Mask used to hide part of image
    * @param feature_detector Algorithm to use for features detection ( see http://opencv.willowgarage.com/documentation/cpp/common_interfaces_for_feature_detection_and_descriptor_extraction.html#featuredetector )
    * @param descriptor_detector Algorithm to use for descriptors detection ( see http://opencv.willowgarage.com/documentation/cpp/common_interfaces_for_feature_detection_and_descriptor_extraction.html#descriptorextractor )
    */
    PointsToTrackWithImage( int corresponding_image,cv::Mat imageToAnalyse,
      cv::Mat maskOfAnalyse, cv::Ptr<cv::FeatureDetector> feature_detector=0,
      cv::Ptr<cv::DescriptorExtractor> descriptor_detector=0 );
    /**
    * Second constructor used to create a list of points to track using a feature and a descriptor algorithm.
    * @param corresponding_image Global index of image
    * @param imageToAnalyse Image to use for keypoints and features search
    * @param maskOfAnalyse Mask used to hide part of image
    * @param feature_detector name of the algorithm to use for features detection ( see http://opencv.willowgarage.com/documentation/cpp/common_interfaces_for_feature_detection_and_descriptor_extraction.html#featuredetector )
    * @param descriptor_detector name of the algorithm to use for descriptors detection ( see http://opencv.willowgarage.com/documentation/cpp/common_interfaces_for_feature_detection_and_descriptor_extraction.html#descriptorextractor )
    */
    PointsToTrackWithImage( int corresponding_image, cv::Mat imageToAnalyse,
      cv::Mat maskOfAnalyse, std::string feature_detector,
      std::string descriptor_detector="SIFT" );
    ~PointsToTrackWithImage( void );
    
    /**
    * Use this function to set the feature detector. Can be useful to update parameters, for example!
    * @param feature_detector new pointer of a feature detector algorithm.
    */
    void setFeatureDetector( cv::Ptr<cv::FeatureDetector> feature_detector );
    /**
    * Use this function to set the descriptor extractor. Can be useful to update parameters, for example!
    * @param descriptor_detector new pointer of a descriptor extractor algorithm.
    */
    void setDescriptorExtractor( cv::Ptr<cv::DescriptorExtractor> descriptor_detector );
    /**
    * This method is used to get color for each points...
    */
    void getColorOfPoints();

    inline cv::Mat getImage( ){return imageToAnalyse_;};
  };

}

#endif
