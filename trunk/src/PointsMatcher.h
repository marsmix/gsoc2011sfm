#ifndef _GSOC_SFM_POINTSMATCHED_H
#define _GSOC_SFM_POINTSMATCHED_H 1

#include "PointsToTrack.h"

#include "opencv2/features2d/features2d.hpp"
#include <vector>

namespace OpencvSfM{
  /*! \brief A class used for matching descriptors that can be described as vectors in a finite-dimensional space
  *
  * Any Matcher that inherit from DescriptorMatcher can be used (For example, you can use FlannBasedMatcher or BruteForceMatcher).
  */
  class PointsMatcher
  {
  public:
    /**
    * Constructor. Need a matcher algorithm...
    * @param matcher Ptr on a matcher. See for available matcher: http://opencv.willowgarage.com/documentation/cpp/features2d_common_interfaces_of_descriptor_matchers.html#descriptormatcher
    */
    PointsMatcher( const cv::Ptr<cv::DescriptorMatcher>& matcher );
    /**
    * Destructor...
    */
    virtual ~PointsMatcher();
    /**
    * Use this function to add data used to find matches
    * @param pointCollection points computed using various methods. Please be carful to get compatible points (that is with descriptors if matcher need some)
    */
    virtual void add( cv::Ptr<PointsToTrack> pointCollection );
    /**
    * If needed, you can clear the training data using this method.
    */
    virtual void clear();
    /**
    * When using matcher which need training, use this method to start the training.
    */
    virtual void train();
    /**
    * Use this method to know if mask are supported with current matcher
    * @return true if matcher can use mask
    */
    virtual bool isMaskSupported();
    /**
    * Use to know if matching are available
    * @return true if matching has been performed
    */
    virtual bool empty() const;
    /**
    * Clone the matcher.
    * @param emptyTrainData – If emptyTrainData is false the method create deep copy of the object, i.e. copies both parameters and train data. If emptyTrainData is true the method create object copy with current parameters but with empty train data..
    * @return An other PointsMatcher instance
    */
    virtual cv::Ptr<PointsMatcher> clone( bool emptyTrainData=true ) const;
    /**
    * Find the k best matches for each descriptor from a query set with train descriptors.
    * @param queryPoints  Query set of points and descriptors.
    * @param matches Mathes. If some query descriptor (keypoint) masked out in mask no match will
    * be added for this descriptor. So matches size may be less than query keypoints count.
    * @param masks The set of masks. Each masks[i] specifies permissible matches between input
    * query keypoints and stored train keypointss from i-th image.
    */
    virtual void match( cv::Ptr<PointsToTrack> queryPoints,
      std::vector<cv::DMatch>& matches,
      const std::vector<cv::Mat>& masks = std::vector<cv::Mat>() );
    /**
    * Find the k best matches for each descriptor from a query set with train descriptors.
    * @param queryPoints  Query set of points and descriptors.
    * @param matches Mathes. Each matches[i] is k or less matches for the same query descriptor.
    * @param k Count of best matches will be found per each query descriptor (or less if it’s not possible).
    * @param masks specifying permissible matches between input query and train matrices of descriptors.
    * @param compactResult – It’s used when mask (or masks) is not empty. If compactResult is false
    * matches vector will have the same size as queryDescriptors rows.
    * If compactResult is true matches vector will not contain matches
    * for fully masked out query descriptors.
    */
    virtual void knnMatch( cv::Ptr<PointsToTrack> queryPoints,
      std::vector<std::vector<cv::DMatch> >& matches, int k,
      const std::vector<cv::Mat>& masks = std::vector<cv::Mat>(), bool compactResult = true );
    /**
    * Find the best matches for each query descriptor which have distance less than given threshold.
    * @param queryPoints  Query set of points and descriptors.
    * @param matches Each matches[i] is k or less matches for the same query descriptor.
    * @param maxDistance The threshold to found match distances.
    * @param masks specifying permissible matches between input query and train matrices of descriptors.
    * @param compactResult – It’s used when mask (or masks) is not empty. If compactResult is false matches
    * vector will have the same size as queryDescriptors rows. If compactResult is true matches vector will
    * not contain matches for fully masked out query descriptors.
    */
    virtual void radiusMatch( cv::Ptr<PointsToTrack> queryPoints,std::vector<std::vector<cv::DMatch> >& matches, float maxDistance,
      const std::vector<cv::Mat>& masks = std::vector<cv::Mat>(), bool compactResult = true );
    /**
    * Using an other matchers given in parameters, recompute a matching in inverse order
    * and keep only matches which are two-ways.
    * @param otherMatcher Query set of points and descriptors.
    * @param matches First guess of matches... Will be updated to contain only
    * two-way matches (can be empty).
    * @param masks specifying permissible matches between input query and train matrices of descriptors.
    */
    virtual void crossMatch( cv::Ptr<PointsMatcher> otherMatcher,
      std::vector<cv::DMatch>& matches,
      const std::vector<cv::Mat>& masks = std::vector<cv::Mat>() );

  protected:

    cv::Ptr<cv::DescriptorMatcher> matcher_;///<Algorithm used to find matches...
    std::vector<cv::Ptr<PointsToTrack>> pointCollection_;///<Vector of points used to compute matches...
  };

}
#endif