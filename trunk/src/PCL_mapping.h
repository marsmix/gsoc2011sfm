
#ifndef PCL_MAPPING_H
#define PCL_MAPPING_H

#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
/*
type not yet taken into account:
Normal : float[4] + float[4]
PointNormal : float[4] * 3
PointWithViewpoint : float[4] + float[4]
PointXYZRGBNormal : (same as before)
PointXYZINormal : (same as before)
*/

/*
cv::Matx can't be used in an union (due to constructors)
As cv::Matx's datas are allocated in stack (as PCL too),
we have to find a way to use union or use bad cast like I do here:
*/
namespace OpencvSfM{
  
  struct EIGEN_ALIGN16 Point2D_mapping
  {
    float *data_;//Max size of points in both library
    bool should_remove;
    unsigned char size_of_data;
    /**
    * Init data using the max size of points in both library (6 floats)
    **/
    Point2D_mapping(){ size_of_data = 6;
    data_ = new float[size_of_data]; should_remove=true; };
    
    /**
    * Init data using previously allocated buffer (size should be >= 6 floats)
    * @param data values of point to convert
    * @param sizeOfBuf in number of float, the size of point
    **/
    Point2D_mapping(float *data, int sizeOfBuf = 6){
      size_of_data = sizeOfBuf; data_ = data; should_remove=false; };

    Point2D_mapping(cv::KeyPoint& kp){
      size_of_data = 6;
      data_ = reinterpret_cast< float* >( &kp );
      should_remove=false; };

    Point2D_mapping(pcl::PointXY& pXY){size_of_data = 2;
      data_ = reinterpret_cast< float* >( &pXY );
      should_remove=false; };
    ~Point2D_mapping(){ if(should_remove) delete data_; };

    inline void setDistinctMemory()
    {
      int newSize = MAX( 6, size_of_data );
      float* data = new float[newSize];
      memcpy( data, data_, size_of_data*sizeof( float ) );
      if(should_remove) delete data_;
      size_of_data = newSize;
      data_ = data;
    }

    //Conversions operators / to pointer:
    template<typename Type, int size>
    inline operator cv::Matx<Type,size,1>*() { CV_Assert( size_of_data >= size );
      return reinterpret_cast< cv::Matx<Type,size,1>* >(data_);};
    template<typename Type, int size>
    inline operator cv::Vec<Type,size>*() { CV_Assert( size_of_data >= size );
      return reinterpret_cast< cv::Vec<Type,size>* >(data_);};
    inline operator cv::KeyPoint*() { CV_Assert( size_of_data >= 6 );
      return reinterpret_cast< cv::KeyPoint* >(data_);};

    inline operator pcl::PointXY*() { CV_Assert( size_of_data >= 2 );
      return reinterpret_cast< pcl::PointXY* >(data_);};

      //Conversions operators / to reference:
    template<typename Type, int size>
    inline operator cv::Matx<Type,size,1>&() { CV_Assert( size_of_data >= size );
      return * reinterpret_cast< cv::Matx<Type,size,1>* >(data_);};
    template<typename Type, int size>
    inline operator cv::Vec<Type,size>&() { CV_Assert( size_of_data >= size );
      return * reinterpret_cast< cv::Vec<Type,size>* >(data_);};
    inline operator cv::KeyPoint&() { CV_Assert( size_of_data >= 6 );
      return * reinterpret_cast< cv::KeyPoint* >(data_);};

    inline operator pcl::PointXY&() { CV_Assert( size_of_data >= 2 );
      return * reinterpret_cast< pcl::PointXY* >(data_);};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct EIGEN_ALIGN16 Point3D_mapping
  {
    float *data_;//Max size of points in both library
    bool should_remove;
    unsigned char size_of_data;
    /**
    * Init data using the max size of points in both library (8 floats)
    **/
    Point3D_mapping(){ size_of_data = 8;
    data_ = new float[size_of_data]; should_remove=true; };
    
    /**
    * Init data using previously allocated buffer (size should be >= 4 floats)
    * @param data values of point to convert
    * @param sizeOfBuf in number of float, the size of point
    **/
    Point3D_mapping(float *data, int sizeOfBuf = 4){
      size_of_data = sizeOfBuf; data_ = data; should_remove=false; };

    template<typename Type, int size>
    Point3D_mapping(cv::Vec<Type,size>& v){
      size_of_data = size*sizeof(Type) / sizeof( float );
 /*     if( size_of_data < 4 )
        CV_Error( CV_StsBadFunc, "Can't deal with opencv type having less than 4 float values.\nPlease use cv::Vec4f instead!");
        */
      data_ = v.val;
      should_remove=false; };

    Point3D_mapping(pcl::PointXYZ& pXYZ){size_of_data = 4;
      data_ = pXYZ.data;
      should_remove=false; };
    ~Point3D_mapping(){ if(should_remove) delete data_; };

    //Conversions operators / to pointer:
    template<typename Type, int size>
    inline operator cv::Matx<Type,size,1>*() { CV_Assert( size_of_data >= size );
      return reinterpret_cast< cv::Matx<Type,size,1>* >(data_);};
    template<typename Type, int size>
    inline operator cv::Vec<Type,size>*() { CV_Assert( size_of_data >= size );
      return reinterpret_cast< cv::Vec<Type,size>* >(data_);};

    inline operator pcl::PointXYZ*() { CV_Assert( size_of_data >= 8 );
      return reinterpret_cast< pcl::PointXYZ* >(data_);};
    inline operator pcl::PointXYZI*() { CV_Assert( size_of_data >= 8 );
      return reinterpret_cast< pcl::PointXYZI* >(data_);};
    inline operator pcl::InterestPoint*() { CV_Assert( size_of_data >= 8 );
      return reinterpret_cast< pcl::InterestPoint* >(data_);};
    inline operator pcl::PointWithRange*() { CV_Assert( size_of_data >= 8 );
      return reinterpret_cast< pcl::PointWithRange* >(data_);};
    inline operator pcl::PointXYZRGBA*() { CV_Assert( size_of_data >= 8 );
      return reinterpret_cast< pcl::PointXYZRGBA* >(data_);};
    inline operator pcl::_PointXYZRGB*() { CV_Assert( size_of_data >= 8 );
      return reinterpret_cast< pcl::_PointXYZRGB* >(data_);};

      //Conversions operators / to reference:
    template<typename Type, int size>
    inline operator cv::Matx<Type,size, 1>&() { CV_Assert( size_of_data >= size );
      return * reinterpret_cast< cv::Matx< Type,size,1 >* >(data_);};
    template<typename Type, int size>
    inline operator cv::Vec<Type,size>&() { CV_Assert( size_of_data >= size );
      return * reinterpret_cast< cv::Vec<Type,size>* >(data_);};

    inline operator pcl::PointXYZ&() { CV_Assert( size_of_data >= 8 );
      return * reinterpret_cast< pcl::PointXYZ* >(data_);};
    inline operator pcl::PointXYZI&() { CV_Assert( size_of_data >= 8 );
      return * reinterpret_cast< pcl::PointXYZI* >(data_);};
    inline operator pcl::InterestPoint&() { CV_Assert( size_of_data >= 8 );
      return * reinterpret_cast< pcl::InterestPoint* >(data_);};
    inline operator pcl::PointWithRange&() { CV_Assert( size_of_data >= 8 );
      return * reinterpret_cast< pcl::PointWithRange* >(data_);};
    inline operator pcl::PointXYZRGBA&() { CV_Assert( size_of_data >= 8 );
      return * reinterpret_cast< pcl::PointXYZRGBA* >(data_);};
    inline operator pcl::_PointXYZRGB&() { CV_Assert( size_of_data >= 8 );
      return * reinterpret_cast< pcl::_PointXYZRGB* >(data_);};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  template<typename TypeFrom, typename TypeTo>
  void convertToMappedVector(const std::vector<TypeFrom>& vectFrom,
    std::vector<TypeTo>& vectTo)
  {
    vectTo.clear();
    vectFrom::iterator it = vectFrom.begin(),
      end = vectFrom.end();

    while ( it != end )
    {
      vectTo.push_back( TypeTo( *it ) );
    }
  }
  template<typename TypeFrom, typename TypeTo>
  void convertFromMappedVector(const std::vector<TypeFrom>& vectFrom,
    std::vector<TypeTo>& vectTo)
  {
    vectTo.clear();
    vectFrom::iterator it = vectFrom.begin(),
      end = vectFrom.end();

    while ( it != end )
    {
      vectTo.push_back( reinterpret_cast< TypeTo >( *it ) );
    }
  }

}
#endif