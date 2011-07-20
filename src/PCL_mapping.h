
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

  namespace mapping{

    /**
    * This function can be used to convert an Matx to a (float*).
    * Be careful that using this function will set is_dynamic_value to false
    * @param mapped pointer on Point2D or Point3D
    * @param t Opencv Matrix to convert
    * @return new array of float values
    */
    template<typename Map_type, typename OpencvType>
    float* convert_to_float(Map_type* mapped, OpencvType t)
    {
      CV_Assert( OpencvType::cols == 1 );
      mapped->is_dynamic_value = false;
      //we have to convert data:
      float* out = new float[ OpencvType::rows ];
      for(int i = 0; i < OpencvType::rows; ++i)
        out[i] = (float) t.val[i];
      return out;
    }

    /**
    * This function can be used to convert quickly a mapped type
    * to any other supported type. No particular checks on data compatibilities
    * are done so be careful to use this function on compatible datas.
    * Checks are only done to wanted size / available data size and if
    * data are directly usable or not.
    * @param mapped pointer on Point2D or Point3D
    * @return pointer on mapped new type
    */
    template<typename Type, typename Map_type>
    inline Type* convert_without_cast( Map_type* mapped )
    {
      CV_Assert( mapped->size_of_data >= sizeof( Type ) / sizeof(float) );
      //data which need direct conversion have to be dynamic...
      CV_Assert( mapped->is_dynamic_value );
      return reinterpret_cast< Type* >( mapped->data_ );
    }

    /**
    * This function can be used to init all datas of a mapped object
    * @param mapped pointer on Point2D or Point3D
    * @param sizeOfBuf (optional) wanted buffer size
    * @param data (optional) existing buffer to map
    */
    template<typename Map_type>
    inline void initData( Map_type* mapped,
      int sizeOfBuf = 0, float *data = NULL)
    {
      mapped->is_dynamic_value = true;
      if( sizeOfBuf <= 0 || data == NULL )
      {
        if( sizeOfBuf <= 0 )
          mapped->size_of_data = 8;
        mapped->data_ = new float[ mapped->size_of_data ];
        mapped->should_remove=true;
      }
      else
      {
        mapped->size_of_data = (unsigned int) sizeOfBuf;
        mapped->data_ = data;
        mapped->should_remove=false;
      }
    }

    /**
    * When you want to convert between 2 types having different footprint size,
    * use this function before to create a buffer with enough space
    * @param mapped pointer on Point2D or Point3D
    */
    template<typename Map_type>
    inline void setDistinctMemory( Map_type* mapped )
    {
      mapped->is_dynamic_value = false;
      int newSize = MAX( 6, size_of_data );
      float* data = new float[newSize];
      memcpy( data, mapped->data_, mapped->size_of_data*sizeof( float ) );
      if( mapped->should_remove ) delete mapped->data_;
      mapped->size_of_data = newSize;
      mapped->data_ = data;
    }


    struct EIGEN_ALIGN16 Point2D
    {
      float *data_;//Max size of points in both library
      bool should_remove;
      bool is_dynamic_value;
      unsigned char size_of_data;
      /**
      * Init data using the max size of points in both library (6 floats)
      **/
      Point2D(){ initData(this, 6); };

      /**
      * Init data using previously allocated buffer (size should be >= 6 floats)
      * @param data values of point to convert
      * @param sizeOfBuf in number of float, the size of point
      **/
      Point2D(float *data, int sizeOfBuf = 6){ initData(this, sizeOfBuf, data); };

      Point2D(cv::KeyPoint& kp){
        initData(this, 6, reinterpret_cast< float* >( &kp ) );
      };

      Point2D(pcl::PointXY& pXY){
        initData(this, 2, reinterpret_cast< float* >( &pXY ) );
      };
      ~Point2D(){ if(should_remove) delete data_; };

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

    struct EIGEN_ALIGN16 Point3D
    {
      float *data_;//Max size of points in both library
      unsigned char size_of_data;
      bool should_remove;
      bool is_dynamic_value;//if true, data_ can be used directly


      /**
      * Init data using the max size of points in both library (8 floats)
      **/
      Point3D(){ initData( this ); };

      /**
      * Init data using previously allocated buffer
      * @param data values of point to convert
      * @param sizeOfBuf in number of float, the size of point
      **/
      Point3D(float *data, int sizeOfBuf = 4){
        initData( this, sizeOfBuf, data );
      };

      template<typename Type, int size>
      Point3D(cv::Vec<Type,size>& v){
        if( sizeof( Type::value_type ) == sizeof( float ) )
          initData( this, Type::rows, (float*) v.val );
        else
        {
          initData( this, Type::rows, convert_to_float(v) );
          should_remove=true;//convert_to_float create a buffer
          is_dynamic_value = false;//data was converted
        }
      };

      template<typename Type, int size>
      Point3D(cv::Matx<Type,size,1>& v){
        if( sizeof( Type::value_type ) == sizeof( float ) )
          initData( this, Type::rows, (float*) v.val );
        else
        {
          initData( this, Type::rows, convert_to_float(v) );
          should_remove=true;//convert_to_float create a buffer
          is_dynamic_value = false;//data was converted
        }
      };

      Point3D(pcl::PointXYZ& pXYZ){ initData( this, 4, pXYZ.data ); };
      Point3D(pcl::PointXYZI& pXYZi){ initData( this, 8, pXYZi.data ); };
      Point3D(pcl::InterestPoint& iP){ initData( this, 8, iP.data ); };
      Point3D(pcl::PointWithRange& pPWR){ initData( this, 8, pPWR.data ); };
      Point3D(pcl::PointXYZRGBA& pXYZ1){ initData( this, 8, pXYZ1.data ); };
      Point3D(pcl::PointXYZRGB& pXYZ2){ initData( this, 8, pXYZ2.data ); };

      ~Point3D(){ if(should_remove) delete data_; };


      //Conversions operators / to reference:
      template<typename Type, int size>
      inline operator cv::Matx<Type,size,1>&() {
        return * convert_without_cast< cv::Matx<Type,size,1> >( this );
      };
      template<typename Type, int size>
      inline operator cv::Vec<Type,size>&() {
        return * convert_without_cast< cv::Vec<Type,size> >( this );
      };
      template<typename Type>
      inline operator cv::Point3_<Type>&() {
        return * convert_without_cast< cv::Point3_<Type> >( this );
      };


      inline operator pcl::PointXYZ&() {
        return * convert_without_cast< pcl::PointXYZ >( this );
      };
      inline operator pcl::PointXYZI&() {
        return * convert_without_cast< pcl::PointXYZI >( this );
      };
      inline operator pcl::InterestPoint&() {
        return * convert_without_cast< pcl::InterestPoint >( this );
      };
      inline operator pcl::PointWithRange&() {
        return * convert_without_cast< pcl::PointWithRange >( this );
      };
      inline operator pcl::PointXYZRGBA&() {
        return * convert_without_cast< pcl::PointXYZRGBA >( this );
      };
      inline operator pcl::PointXYZRGB&() {
        return * convert_without_cast< pcl::PointXYZRGB >( this );
      };


      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    template<typename TypeFrom, typename TypeTo>
    void convertToMappedVector(const std::vector<TypeFrom>& vectFrom,
      std::vector<TypeTo>& vectTo)
    {
      vectTo.clear();
      unsigned int it = 0,
        end = vectFrom.size();

      while ( it < end )
      {
        vectTo.push_back( TypeTo( vectFrom[it] ) );
        it++;
      }
    }
    template<typename TypeFrom, typename TypeTo>
    void convertFromMappedVector(const std::vector<TypeFrom>& vectFrom,
      std::vector<TypeTo>& vectTo)
    {
      vectTo.clear();
      unsigned int it = 0,
        end = vectFrom.size();

      while ( it < end )
      {
        vectTo.push_back( reinterpret_cast< TypeTo >( vectFrom[it] ) );
        it++;
      }
    }

  }

}
#endif
