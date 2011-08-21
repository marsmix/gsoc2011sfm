#ifndef _GSOC_SFM_BOOST_THREAD_H
#define _GSOC_SFM_BOOST_THREAD_H 1

#include "macro.h" //SFM_EXPORTS

#include <vector>
#include "opencv2/core/core.hpp"

#include "SequenceAnalyzer.h"

namespace OpencvSfM{

  /**
  *  \brief This struct is used by boost::thread object to compute match.
  * I used some semaphore to ensure the matching process work well.
  */
  struct MatchingThread{
    unsigned int i;///<Index of source image. This image will be matched against every other
    std::vector< cv::Ptr< PointsToTrack > >::iterator matches_it;///<iterator of every Points for track (points of other images to match)
    cv::Ptr<SequenceAnalyzer> seq_analyser;///<This object contains every sequence related info (images, points, tracks...)

    static std::vector< cv::Ptr< PointsToTrack > >::iterator end_matches_it;///<End of list images of points. It's the same for every thread, so set once for every thread before runing computation.
    static std::vector< cv::Mat > masks;///<List of mask to hide some points in the matching computation.
    static unsigned int mininum_points_matches;///<Minimum matches between two images to accept the matches
    static PointsMatcher* match_algorithm;///<The algorithm to use for matching.
    //semaphore to synchronize threads:
    CREATE_STATIC_MUTEX( thread_concurr );///<Used to start as many thread as processors
    CREATE_STATIC_MUTEX( thread_unicity );///<Used around critical sections

    /**
    * Constructor of a thread.
    * @param seq_analyser the sequence related infos
    * @param i Index of source image. This image will be matched against every other
    * @param matches_it iterator of every Points for track (points of other images to match)
    */
    MatchingThread(cv::Ptr<SequenceAnalyzer> seq_analyser,unsigned int i,
      std::vector< cv::Ptr< PointsToTrack > >::iterator matches_it);

    /**
    * Thread implementation...
    */
    void operator()();
  };


}

#endif