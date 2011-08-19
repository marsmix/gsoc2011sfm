#ifndef _GSOC_SFM_STRUCTURE_ESTIMATOR_H
#define _GSOC_SFM_STRUCTURE_ESTIMATOR_H 1

#include <vector>

#include "macro.h" //SFM_EXPORTS

namespace OpencvSfM{
  class SFM_EXPORTS SequenceAnalyzer;
  class SFM_EXPORTS PointOfView;
  class SFM_EXPORTS TrackOfPoints;
  /**
  * \brief This class tries to find the 3D structure
  * using a sequence and cameras fully parameterized
  */
  class SFM_EXPORTS StructureEstimator
  {
  protected:
    SequenceAnalyzer *sequence_;
    std::vector<PointOfView>* cameras_;
    int max_repro_error_;

  public:
    StructureEstimator( SequenceAnalyzer *sequence,
      std::vector<PointOfView>* cameras,int max_repro_error=10 )
      :sequence_( sequence ),cameras_( cameras ),
      max_repro_error_( max_repro_error ){};

    /**
    * Destructor will not release datas as they where given by address!
    */
    ~StructureEstimator(){sequence_=NULL;cameras_=NULL;};
    
    /**
    * Project previously 2D points matches using cameras parameters
    * @return the mask of correct points ( 0 if the point can't be triangulate )
    */
    std::vector<char> computeStructure( unsigned int max_error = 10 );
    /**
    * Project previously 2D points matches for only two views
    * @param img1 first image to use
    * @param img1 second image to use
    * @return output of tracks triangulated ( contain 3D point )
    */
    std::vector< TrackOfPoints > computeStructure(
      const std::vector<int>& list_of_images,
       unsigned int max_error = 10 );
    /**
    * Remove points from track when projection error > max_error 
    * @param min_error maximum error of back projection allowed
    */
    void removeOutliersTracks( double max_error = 10 );
  };

}



#endif