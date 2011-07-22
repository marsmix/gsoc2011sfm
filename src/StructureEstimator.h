#ifndef _GSOC_SFM_STRUCTURE_ESTIMATOR_H
#define _GSOC_SFM_STRUCTURE_ESTIMATOR_H 1

#include "macro.h" //SFM_EXPORTS
#include "SequenceAnalyzer.h"

namespace OpencvSfM{
  
  /**
  * \brief This class tries to find the 3D structure
  * using a sequence and cameras fully parameterized
  */
  class SFM_EXPORTS StructureEstimator
  {
  protected:
    SequenceAnalyzer &sequence_;
    std::vector<PointOfView>& cameras_;
    int max_repro_error_;

  public:
    StructureEstimator(SequenceAnalyzer &sequence,
      std::vector<PointOfView>& cameras,int max_repro_error=10)
      :sequence_(sequence),cameras_(cameras),
      max_repro_error_(max_repro_error){};
    
    /**
    * Project previously 2D points matches using cameras parameters
    * @return the mask of correct points (0 if the point can't be triangulate)
    */
    std::vector<char> computeStructure();
    /**
    * Project previously 2D points matches for only two views
    * @param img1 first image to use
    * @param img1 second image to use
    * @param points3D output of points in 3D
    */
    void computeTwoView(int img1, int img2,
      std::vector<TrackOfPoints>& points3D);

  };

}


#endif