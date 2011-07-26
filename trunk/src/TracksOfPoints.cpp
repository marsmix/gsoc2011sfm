
#include "TracksOfPoints.h"

namespace OpencvSfM{
  using cv::KeyPoint;
  using std::vector;
  using cv::Point3d;
  using cv::DMatch;
  using cv::Ptr;
  using cv::Mat;

  inline void quickSort(vector<ImageLink>& outLinks,
    vector<int>& arr, int left, int right) {
      int i = left, j = right;
      int tmp;
      int pivot = arr[(left + right) / 2];

      /* partition */
      while (i <= j) {
        while (arr[i] < pivot)
          i++;
        while (arr[j] > pivot)
          j--;
        if (i < j) {
          tmp = arr[i];
          arr[i] = arr[j];
          arr[j] = tmp;

          ImageLink tmp1 = outLinks[i];
          outLinks[i] = outLinks[j];
          outLinks[j] = tmp1;

        }
        if (i <= j) {
          i++;
          j--;
        }
      };

      /* recursion */
      if (left < j)
        quickSort(outLinks, arr, left, j);
      if (i < right)
        quickSort(outLinks, arr, i, right);
  }

  bool TrackOfPoints::addMatch(const int image_src, const int point_idx1)
  {
    if( track_consistance<0 )
      return false;

    //If a track contains more than one keypoint in the same image but
    //a different keypoint, it is deemed inconsistent.
    vector<unsigned int>::iterator indexImg = images_indexes_.begin();
    vector<unsigned int>::iterator end_iter = images_indexes_.end();
    unsigned int index=0;
    while(indexImg != end_iter)
    {
      if ( *indexImg == image_src )
      {
        if( point_indexes_[index] == point_idx1 )
        {
          if( track_consistance>=0 )
            track_consistance++;
        }
        else
          track_consistance=-1;

        return track_consistance>=0;
      }
      index++;
      indexImg++;
    }

    images_indexes_.push_back(image_src);
    point_indexes_.push_back(point_idx1);
    return track_consistance>=0;
  }

  bool TrackOfPoints::containPoint(const int image_src, const int point_idx1)
  {
    //we don't use find here because we want the number instead of iterator...
    vector<unsigned int>::iterator indexImg = images_indexes_.begin();
    vector<unsigned int>::iterator end_iter = images_indexes_.end();
    unsigned int index=0;
    while(indexImg != end_iter)
    {
      if ( *indexImg == image_src )
      {
        if( point_indexes_[index] == point_idx1 )
          return true;
      }
      index++;
      indexImg++;
    }
    return false;
  }

  DMatch TrackOfPoints::toDMatch(const int img1,const int img2) const
  {
    DMatch outMatch;
    char nbFound=0;
    //we don't use find here because we want the number instead of iterator...
    vector<unsigned int>::const_iterator indexImg = images_indexes_.begin();
    vector<unsigned int>::const_iterator end_iter = images_indexes_.end();
    unsigned int index=0;
    while(indexImg != end_iter)
    {
      if ( *indexImg == img1 )
      {
        nbFound++;
        outMatch.trainIdx = point_indexes_[index];
        if(nbFound==2)
          return outMatch;
      }
      if ( *indexImg == img2 )
      {
        nbFound++;
        outMatch.queryIdx = point_indexes_[index];
        if(nbFound==2)
          return outMatch;
      }
      index++;
      indexImg++;
    }
    return outMatch;
  };

  void TrackOfPoints::getMatch(const unsigned int index,
    int &idImage, int &idPoint) const
  {
    char nbFound=0;
    if( index < images_indexes_.size() )
    {
      idImage = images_indexes_[index];
      idPoint = point_indexes_[index];
    }
  };

  double TrackOfPoints::errorEstimate(std::vector<PointOfView>& cameras,
    const std::vector< cv::Ptr< PointsToTrack > > &points_to_track,
    cv::Vec3d& points3D) const
  {
    double distance=0.0;
    unsigned int nviews = images_indexes_.size();
    for (unsigned int cpt = 0; cpt < nviews; cpt++) {
      int num_camera=images_indexes_[cpt];
      int num_point=point_indexes_[cpt];
      cv::Ptr<PointsToTrack> points2D = points_to_track[ num_camera ];

      const KeyPoint& p=points2D->getKeypoint(num_point);
      cv::Vec2d projP = cameras[num_camera].project3DPointIntoImage(points3D);

      //compute back-projection
      distance += ((p.pt.x-projP[0])*(p.pt.x-projP[0]) + 
        (p.pt.y-projP[1])*(p.pt.y-projP[1]) );
    }
    return distance/static_cast<double>(nviews);
  }
  double TrackOfPoints::triangulateLinear(vector<PointOfView>& cameras,
    const std::vector< cv::Ptr< PointsToTrack > > &points_to_track,
    cv::Vec3d& points3D, const vector<bool> &masks)
  {
    unsigned int nviews = 0;
    bool hasMask=false;
    unsigned int i;
    if( masks.size()==images_indexes_.size() )
    {
      for (i=0; i<images_indexes_.size(); ++i)
      {
        if(masks[i]!=0)
          nviews++;
      }
      hasMask=true;
    }else
    {
      nviews = images_indexes_.size();
    }

    Mat design = Mat::zeros(3*nviews, 4 + nviews,CV_64FC1);
    unsigned  int real_position=0;
    i = 0;
    for (real_position = 0; real_position < images_indexes_.size();
      ++real_position ) {
        if( !hasMask || ( hasMask && masks[real_position]!=0))
        {
          int num_camera=images_indexes_[ real_position ];
          int num_point=point_indexes_[ real_position ];
          cv::Ptr<PointsToTrack> points2D = points_to_track[ num_camera ];
          const KeyPoint& p=points2D->getKeypoint(num_point);

          design(cv::Range(3*i,3*i+3), cv::Range(0,4)) = 
            -cameras[num_camera].getProjectionMatrix();
          design.at<double>(3*i + 0, 4 + i) = p.pt.x;
          design.at<double>(3*i + 1, 4 + i) = p.pt.y;
          design.at<double>(3*i + 2, 4 + i) = 1.0;
          i++;
        }
    }
    Mat X_and_alphas;
    cv::SVD::solveZ(design, X_and_alphas);

    double scal_factor=X_and_alphas.at<double>(3,0);
    points3D[0]=X_and_alphas.at<double>(0,0)/scal_factor;
    points3D[1]=X_and_alphas.at<double>(1,0)/scal_factor;
    points3D[2]=X_and_alphas.at<double>(2,0)/scal_factor;

    //update the point 3D:
    if(point3D.empty())
      point3D = Ptr<cv::Vec3d>( new cv::Vec3d(points3D) );
    else
      *point3D = points3D;

    return errorEstimate(cameras, points_to_track, points3D);
  }

  double TrackOfPoints::triangulateRobust(std::vector<PointOfView>& cameras,
    const std::vector< cv::Ptr< PointsToTrack > > &points_to_track,
    cv::Vec3d& points3D, double reproj_error,
    const std::vector<bool> &masksValues)
  {
    cv::RNG& rng = cv::theRNG();
    unsigned int nviews = images_indexes_.size();
    double distance=0, best_distance=1e20;
    vector<bool> masks;
    cv::Vec3d bestPoints3D;

    if( !point3D.empty() )
    {
      double error = errorEstimate(cameras, points_to_track, *point3D);
      if( error<reproj_error )
      {
        points3D = *point3D;
        return error;
      }
    }
    int ptSize = 0;
    if( masksValues.size() == nviews )
    {
      for (int i=0; i<nviews; ++i)
      {
        if(masksValues[i]!=0)
          ptSize++;
      }
    }
    else
      ptSize = nviews;

    int num_iter=0, max_iter=ptSize - 1;
    for(num_iter=0; num_iter<max_iter; ++num_iter)
    {
      masks.clear();
      int nb_vals=0;
      for (unsigned int cpt = 0; cpt < nviews; cpt++) {
        bool valTmp = (rng(2) != 0);
        if( masksValues.size() == nviews )
          valTmp = valTmp & masksValues[cpt];
        if( valTmp )
          nb_vals++;
        masks.push_back(valTmp);
      }
      while( nb_vals<2 )
      {
        int valTmp = rng(nviews);
        if( masks[valTmp] == 0 )
        {
          masks[valTmp] = 1;
          nb_vals++;
        }
      }
      //create mask:
      triangulateLinear( cameras, points_to_track, points3D, masks);

      distance = errorEstimate(cameras, points_to_track, points3D);
      if( distance < best_distance )
      {
        //new best model...
        bestPoints3D = points3D;
        best_distance = distance;
        if( best_distance<reproj_error )
          num_iter=nviews;//quit the loop!
      }
    }
    points3D = bestPoints3D;
    //update the point 3D:
    if(point3D.empty())
      point3D = Ptr<cv::Vec3d>( new cv::Vec3d(points3D) );
    else
      *point3D = points3D;
    return distance;
  }
  
  int ImagesGraphConnection::getHighestLink(int &first_image, int &second_image,
    int max_number)
  {
    cv::SparseMatConstIterator
      it = images_graph_.begin(),
      it_end = images_graph_.end();
    double s = 0;
    const int dims = 2;
    int max_value=0;
    for(; it != it_end; ++it)
    {
      int currentValue=it.value<int>();
      if( currentValue < max_number && currentValue > max_value )
      {
        max_value = currentValue;
        const cv::SparseMat::Node* n = it.node();
        first_image = n->idx[0];
        second_image = n->idx[1];
      }
    }
    return max_value;
  }


  void ImagesGraphConnection::getOrderedLinks(std::vector<ImageLink>& outLinks,
    int min_number, int max_number)
  {
    cv::SparseMatConstIterator
      it = images_graph_.begin(),
      it_end = images_graph_.end();

    std::vector<int> distance;
    double s = 0;
    const int dims = 2;
    for(; it != it_end; ++it)
    {
      int currentValue=it.value<int>();
      if( currentValue <= max_number && currentValue >= min_number )
      {
        const cv::SparseMat::Node* n = it.node();
        ImageLink link;
        link.imgSrc = n->idx[0];
        link.imgDest = n->idx[1];
        outLinks.push_back(link);
        distance.push_back(currentValue);
      }
    }
    //order the list:
    quickSort(outLinks, distance, 0, distance.size()-1 );
  }

  void ImagesGraphConnection::getImagesRelatedTo(int first_image,
    std::vector<ImageLink>& outList, int min_number, int max_number)
  {
    int it = 0,
      it_end = images_graph_.size()[0];
    int idx[2];
    for(; it < first_image; ++it)
    {
      uchar* currentValue=images_graph_.ptr(it, first_image, false);
      if( currentValue != NULL )
      {
        //we have a value here... Are there enough links:
        int &val = * reinterpret_cast<int*>(currentValue);
        if( val > min_number && val < max_number )
        {
          //it's a new link, add it:
          ImageLink link;
          link.imgSrc = it;
          link.imgDest = first_image;
          outList.push_back(link);
        }
      }
    }
    ++it;//skeep the diagonal
    for(; it < it_end; ++it)
    {
      uchar* currentValue=images_graph_.ptr( first_image, it, false);
      if( currentValue != NULL )
      {
        //we have a value here... Are there enough links:
        int &val = * reinterpret_cast<int*>(currentValue);
        if( val > min_number && val < max_number )
        {
          //it's a new link, add it:
          ImageLink link;
          link.imgSrc = first_image;
          link.imgDest = it;
          outList.push_back(link);
        }
      }
    }

  }
}
