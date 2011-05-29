#ifndef _GSOC_SFM_POINTSMATCHED_H
#define _GSOC_SFM_POINTSMATCHED_H 1

#include "PointsToTrack.h"

#include "opencv2/features2d/features2d.hpp"
#include <vector>

namespace OpencvSfM{
  /*! \brief A class used for matching descriptors that can be described as vectors in a finite-dimensional space
  */
  class PointsMatcher
  {
  public:
    PointsMatcher( const cv::Ptr<cv::DescriptorMatcher>& matcher );
    virtual ~PointsMatcher();

    virtual void add( cv::Ptr<PointsToTrack> pointCollection );

    virtual void clear();

    virtual void train();

    virtual bool isMaskSupported();

    virtual bool empty() const;

    virtual cv::Ptr<PointsMatcher> clone( bool emptyTrainData=false ) const;

    virtual void knnMatch( cv::Ptr<PointsToTrack> queryPoints,std::vector<std::vector<cv::DMatch> >& matches, int k,
      const std::vector<cv::Mat>& masks, bool compactResult );
    virtual void radiusMatch( cv::Ptr<PointsToTrack> queryPoints,std::vector<std::vector<cv::DMatch> >& matches, float maxDistance,
      const std::vector<cv::Mat>& masks, bool compactResult );

  protected:

    cv::Ptr<cv::DescriptorMatcher> matcher_;
    std::vector<cv::Ptr<PointsToTrack>> pointCollection_;
  };

}

#endif