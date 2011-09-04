#ifndef _GSOC_SFM_MATCHERSPARSEFLOW_H
#define _GSOC_SFM_MATCHERSPARSEFLOW_H 1

#include "macro.h" //SFM_EXPORTS

#include "opencv2/features2d/features2d.hpp"
#include <vector>

#include "config_SFM.h"  //SEMAPHORE
#include "PointsMatcher.h"


namespace OpencvSfM{
  
  /*! \brief A class used for matching points between two images
  *
  * This class use first a feature based matcher to have guesses about optical flow
  * and then use calcOpticalFlowPyrLK to find every match (and add points to
  * other image if needed)
  */
  class SFM_EXPORTS MatcherSparseFlow : public PointsMatcher
  {
    double max_distance_;///<In pixel, the threshold to know if a match is close enough to the predicted position
  public:
    /**
    * Construct a new point matcher using the name of the opticalflow algorithm.
    * @param name_of_algo name of the matching algo (see http://opencv.willowgarage.com/documentation/cpp/features2d_common_interfaces_of_descriptor_matchers.html)
    * @param dist_allowed in pixel, the threshold to know if a match is close
    * enough to the predicted position
    */
    MatcherSparseFlow( const cv::Ptr<cv::DescriptorMatcher>& matcher, double dist_allowed = 2.0 );

    /**
    * Use this function to create a point matcher using the name of an opticalflow
    * algorithm (see http://opencv.willowgarage.com/documentation/cpp/features2d_common_interfaces_of_descriptor_matchers.html)
    * @param match_algo name of the wanted algorithm
    * @param dist_allowed in pixel, the threshold to know if a match is close
    * enough to the predicted position
    * @return a new PointsMatcher
    */
    static cv::Ptr<PointsMatcher> create( std::string match_algo, double dist_allowed = 2.0 )
    {
      return cv::Ptr<PointsMatcher>( new MatcherSparseFlow(
        cv::DescriptorMatcher::create( match_algo ), dist_allowed ) );
    };
    /**
    * Clone the matcher.
    * @param emptyTrainData IIf emptyTrainData is false the method create deep copy of the object, i.e. copies both parameters and train data. If emptyTrainData is true the method create object copy with current parameters but with empty train data..
    * @return An other PointsMatcher instance
    */
    virtual cv::Ptr<PointsMatcher> clone( bool emptyTrainData=true );
    /**
    * Find the k best matches for each descriptor from a query set with train descriptors.
    * @param queryPoints  Query set of points and descriptors.
    * @param matches Mathes. If some query descriptor ( keypoint ) masked out in mask no match will
    * be added for this descriptor. So matches size may be less than query keypoints count.
    * @param masks The set of masks. Each masks[ i ] specifies permissible matches between input
    * query keypoints and stored train keypointss from i-th image.
    */
    virtual void match( cv::Ptr<PointsToTrack> queryPoints,
      std::vector<cv::DMatch>& matches,
      const std::vector<cv::Mat>& masks = std::vector<cv::Mat>( ) );
    /**
    * Find the k best matches for each descriptor from a query set with train descriptors.
    * @param queryPoints  Query set of points and descriptors.
    * @param matches Mathes. Each matches[ i ] is k or less matches for the same query descriptor.
    * @param k Count of best matches will be found per each query descriptor ( or less if it’s not possible ).
    * @param masks specifying permissible matches between input query and train matrices of descriptors.
    * @param compactResult – It’s used when mask ( or masks ) is not empty. If compactResult is false
    * matches vector will have the same size as queryDescriptors rows.
    * If compactResult is true matches vector will not contain matches
    * for fully masked out query descriptors.
    */
    virtual void knnMatch( cv::Ptr<PointsToTrack> queryPoints,
      std::vector<std::vector<cv::DMatch> >& matches, int k,
      const std::vector<cv::Mat>& masks = std::vector<cv::Mat>( ), bool compactResult = true );
    /**
    * Find the best matches for each query descriptor which have distance less than given threshold.
    * @param queryPoints  Query set of points and descriptors.
    * @param matches Each matches[ i ] is k or less matches for the same query descriptor.
    * @param maxDistance The threshold to found match distances.
    * @param masks specifying permissible matches between input query and train matrices of descriptors.
    * @param compactResult – It’s used when mask ( or masks ) is not empty. If compactResult is false matches
    * vector will have the same size as queryDescriptors rows. If compactResult is true matches vector will
    * not contain matches for fully masked out query descriptors.
    */
    virtual void radiusMatch( cv::Ptr<PointsToTrack> queryPoints,std::vector<std::vector<cv::DMatch> >& matches, float maxDistance,
      const std::vector<cv::Mat>& masks = std::vector<cv::Mat>( ), bool compactResult = true );
  };

}

#endif
