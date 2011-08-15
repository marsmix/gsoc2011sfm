#ifndef _GSOC_SFM_POINTSMATCHED_H
#define _GSOC_SFM_POINTSMATCHED_H 1

#include "macro.h" //SFM_EXPORTS
#include "config_SFM.h"  //SEMAPHORE

#include "PointsToTrack.h"

#include "opencv2/features2d/features2d.hpp"
#include <vector>

namespace OpencvSfM{
  /*! \brief A class used for matching descriptors that can be described as vectors in a finite-dimensional space
  *
  * Any Matcher that inherit from DescriptorMatcher can be used ( For example, you can use FlannBasedMatcher or BruteForceMatcher ).
  */
  class SFM_EXPORTS PointsMatcher
  {
    /**
    * In order to be able to match points in threads, we have to
    * take care of interprocess access.
    */
    DECLARE_MUTEX( thread_concurr );
  public:
    /**
    * Constructor. Need a matcher algorithm...
    * @param matcher Ptr on a matcher. See for available matcher: http://opencv.willowgarage.com/documentation/cpp/features2d_common_interfaces_of_descriptor_matchers.html#descriptormatcher
    */
    PointsMatcher( const cv::Ptr<cv::DescriptorMatcher>& matcher );

    PointsMatcher( const PointsMatcher& copy );
    /**
    * Destructor...
    */
    virtual ~PointsMatcher( );

    static cv::Ptr<PointsMatcher> create( std::string match_algo )
    {
      return cv::Ptr<PointsMatcher>( new PointsMatcher(
        cv::DescriptorMatcher::create( match_algo ) ) );
    };
    /**
    * Use this function to add data used to find matches
    * @param pointCollection points computed using various methods. Please be carful to get compatible points ( that is with descriptors if matcher need some )
    */
    virtual void add( cv::Ptr<PointsToTrack> pointCollection );
    /**
    * If needed, you can clear the training data using this method.
    */
    virtual void clear( );
    /**
    * When using matcher which need training, use this method to start the training.
    */
    virtual void train( );
    /**
    * Use this method to know if mask are supported with current matcher
    * @return true if matcher can use mask
    */
    virtual bool isMaskSupported( );
    /**
    * Use to know if matching are available
    * @return true if matching has been performed
    */
    virtual bool empty( ) const;
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
    /**
    * Using an other matchers given in parameters, recompute a matching in inverse order
    * and keep only matches which are two-ways.
    * @param otherMatcher Query set of points and descriptors.
    * @param matches First guess of matches... Will be updated to contain only
    * two-way matches ( can be empty ).
    * @param masks specifying permissible matches between input query and train matrices of descriptors.
    */
    virtual void crossMatch( cv::Ptr<PointsMatcher> otherMatcher,
      std::vector<cv::DMatch>& matches,
      const std::vector<cv::Mat>& masks = std::vector<cv::Mat>( ) );
    /**
    * Using an other matchers given in parameters and a fundamental matrix, compute
    * matches which agree with fundamental matrix ( Sampson distance is used )
    * @param otherMatcher Query set of points and descriptors.
    * @param fundamentalMat Matrix of fundamental equation.
    * @param matches [ output ] matches.
    * @param masks specifying permissible matches between input query and train matrices of descriptors.
    */
    void matchWithFundamental( cv::Ptr<PointsMatcher> otherMatcher,
      cv::Mat fundamentalMat,
      cv::Mat img1,
      std::vector<cv::DMatch>& matches,
      const std::vector<cv::Mat>& masks );
    /**
    * This function draw keypoints and matches.
    * Contrary to cv::drawMatches, only the first image is used
    * to draw matches...
    * @param img1 First source image.
    * @param keypoints1 Keypoints from first source image.
    * @param keypoints2 Keypoints from second source image.
    * @param matches Matches from first image to second one,
    * i.e. keypoints1[ i ] has corresponding point keypoints2[ matches[ i ]] .
    * @param outImg Output image. Its content depends on flags value
    * what is drawn in output image. See below possible flags bit values.
    * @param matchColor – Color of matches ( lines and connected keypoints ).
    * If matchColor==Scalar::all( -1 ) color will be generated randomly.
    * @param singlePointColor – Color of single keypoints ( circles ),
    * i.e. keypoints not having the matches. If singlePointColor==Scalar::all( -1 )
    * color will be generated randomly.
    * @param matchesMask – Mask determining which matches will be drawn.
    * If mask is empty all matches will be drawn.
    * @param flags – Each bit of flags sets some feature of drawing.
    * Possible flags bit values is defined by DrawMatchesFlags , see http://opencv.willowgarage.com/documentation/cpp/features2d_drawing_function_of_keypoints_and_matches.html#cv-drawmatches.
    */
    static void drawMatches( const cv::Mat& img1,
      const std::vector<cv::KeyPoint>& keypoints1,
      const std::vector<cv::KeyPoint>& keypoints2,
      const std::vector<cv::DMatch>& matches1to2, cv::Mat& outImg,
      const cv::Scalar& matchColor=cv::Scalar::all( -1 ),
      const cv::Scalar& singlePointColor=cv::Scalar::all( -1 ),
      const std::vector<char>& matchesMask=std::vector<char>( ),
      int flags=cv::DrawMatchesFlags::DEFAULT
      );

    static void read( const cv::FileNode& node, PointsMatcher& points );

    static void write( cv::FileStorage& fs, const PointsMatcher& points );

    inline const cv::KeyPoint &getKeypoint( int numKey ) const
    {
      CV_DbgAssert( pointCollection_.size( )>0 );
      return pointCollection_[ 0 ]->getKeypoint( numKey );
    }

  protected:

    cv::Ptr<cv::DescriptorMatcher> matcher_;///<Algorithm used to find matches...
    std::vector< cv::Ptr< PointsToTrack > > pointCollection_;///<Vector of points used to compute matches...
  };

}

#endif
