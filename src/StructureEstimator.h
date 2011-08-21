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
    SequenceAnalyzer *sequence_;///<Object containing all 2D information of this sequence
    std::vector<PointOfView>* cameras_;///<List of cameras (intra and extern parameters...)
    int max_repro_error_;///<Maximum reprojection error allowed

  public:
    /**
    * Constructor of this 3D structure estimator
    * @param sequence the address of the object containing all 2D information of this sequence
    * @param cameras List of cameras (intra and extern parameters...)
    * @param max_repro_error Maximum reprojection error allowed
    */
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
    * @param max_error maximum error allowed.
    * @return the mask of correct points ( 0 if error > max_error )
    */
    std::vector<char> computeStructure( unsigned int max_error = 10 );
    /**
    * Project previously 2D points matches for only two views
    * @param list_of_images list of image indexes to use
    * @param max_error maximum error allowed.
    * @return output of tracks triangulated ( contain 3D point )
    */
    std::vector< TrackOfPoints > computeStructure(
      const std::vector<int>& list_of_images,
       unsigned int max_error = 10 );
    /**
    * Remove points from track when projection error > max_error 
    * @param max_error maximum error of back projection allowed
    * @param list_of_tracks list of tracks to work with.
    *        If NULL or not set, will use StructureEstimator::sequence_
    */
    void removeOutliersTracks( double max_error = 10,
      std::vector< TrackOfPoints >* list_of_tracks = NULL );
  };

}



#endif