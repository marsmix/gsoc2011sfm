
#ifndef PCL_MAPPING_H
#define PCL_MAPPING_H

#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
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
    * @param mapped pointer on Point2D or Point3D
    * @param t Opencv Matrix to convert
    * @return new array of float values
    */
    template<typename OpencvType>
    float* convert_to_float( const OpencvType& t )
    {
      CV_Assert( OpencvType::cols == 1 );
      //we have to convert data:
      float* out = new float[ OpencvType::rows ];
      for(int i = 0; i < OpencvType::rows; ++i)
        out[i] = (float) t.val[i];
      return out;
    }
    /**
    * This function can be used to convert float values to double values (or int).
    * The function will create a new buffer, you will be responsible of data release
    * @param mapped pointer on Point2D or Point3D
    * @return new array of new Type values
    */
    template<typename Type>
    Type* convert_to_(const float* data, int sizeOfBuf = 8)
    {
      //we have to convert data:
      Type* out = new Type[ sizeOfBuf ];
      for(int i = 0; i < sizeOfBuf; ++i)
        out[i] = (Type) data[i];
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
      if( sizeOfBuf <= 0 || data == NULL )
      {
        if( sizeOfBuf <= 0 )
          mapped->size_of_data = 8;
        else
          mapped->size_of_data = sizeOfBuf;
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
    inline void setDistinctMemory( Map_type* mapped,
      int new_mem_size = 8, float* newData = NULL )
    {
      int newSize = MAX( new_mem_size, mapped->size_of_data );
      float* data = newData;
      if( data == NULL)
      {
        data = new float[newSize];
        memcpy( data, mapped->data_, mapped->size_of_data*sizeof( float ) );
      }
      if( mapped->should_remove ) delete mapped->data_;
      mapped->should_remove = true;
      mapped->size_of_data = newSize;
      mapped->data_ = data;
    }
    /**
    * This function can be used to convert a mapped type
    * to any other supported type. Here we take care of data compatibilities
    * and if needed we create an other buffer to hold data
    * @param mapped pointer on Point2D or Point3D
    * @return pointer on mapped new type
    */
    template<typename LowDataType, typename Type, typename Map_type>
    inline void convert_( Map_type* mapped, Type* out )
    {
      CV_Assert( mapped->size_of_data >= sizeof( Type ) / sizeof(float) );
      LowDataType* data = convert_to_<LowDataType>( mapped->data_, mapped->size_of_data );
      memcpy(reinterpret_cast< char* >( out ), data, sizeof(Type) );
      delete data;
    }

    struct EIGEN_ALIGN16 Point2D
    {
      float *data_;//Max size of points in both library
      bool should_remove;
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
      
      /**
      * Copy constructor (deep copy!)
      **/
      Point3D(const Point3D& otherP){
        initData( this, otherP.size_of_data );
        memcpy( data_, otherP.data_, size_of_data * sizeof(float) );
      };

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
        if( sizeof( Type ) == sizeof( float ) )
          initData( this, v.rows, (float*) v.val );
        else
        {
          initData( this, v.rows, convert_to_float( v ) );
          should_remove=true;//convert_to_float create a buffer
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
        CV_Assert( sizeof(Type) == sizeof( float ) );
        return * convert_without_cast< cv::Matx<Type,size,1> >( this );
      };
      template<typename Type, int size>
      inline operator cv::Vec<Type,size>&() {
        CV_Assert( sizeof(Type) == sizeof( float ) );
        return * convert_without_cast< cv::Vec<Type,size> >( this );
      };
      template<typename Type>
      inline operator cv::Point3_<Type>&() {
        CV_Assert( sizeof(Type) == sizeof( float ) );
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

    template<typename TypeFrom, typename TypeTo,
      typename allocatorFrom, typename allocatorTo>
    void convertToMappedVector(
      const std::vector<TypeFrom,allocatorFrom>& vectFrom,
      std::vector<TypeTo,allocatorTo>& vectTo)
    {
      vectTo.clear();
      unsigned int it = 0,
        end = vectFrom.size();
      vectTo.reserve(end);

      while ( it < end )
      {
        //can't create directly the new value because of the const arg...
        TypeTo convertor(NULL, sizeof( TypeFrom ) / sizeof( float ) );
        const TypeFrom& val = vectFrom[ it ];

        memcpy( reinterpret_cast<char*>( convertor.data_ ),
          reinterpret_cast<const char*>( &val ),
          sizeof( TypeFrom ) );//assign the new value

        vectTo.push_back( convertor );//create an empty value
        it++;
      }
    }

    template<typename TypeFrom, typename TypeTo,
      typename allocatorFrom, typename allocatorTo>
    void convertFromMappedVector(
      std::vector<TypeFrom,allocatorFrom>& vectFrom,
      std::vector<TypeTo,allocatorTo>& vectTo)
    {
      vectTo.clear();
      unsigned int it = 0,
        end = vectFrom.size();
      vectTo.reserve(end);

      while ( it < end )
      {
        vectTo.push_back( TypeTo() );//create an empty value
        int minSize = sizeof( vectTo[ it ] ) / sizeof( float );
        //We must have enough space in mapped memory:
        if( minSize > vectFrom[it].size_of_data )
          setDistinctMemory( &vectFrom[it], minSize );


        memcpy( reinterpret_cast<char*>( &vectTo[ it ] ),
          reinterpret_cast<const char*>( vectFrom[it].data_ ),
          minSize * sizeof( float ) );//assign the new value
        it++;
      }
    }

    template<typename TypeFrom, typename TypeTo,
      typename allocatorFrom, typename allocatorTo>
    void convertVectorFrom_OpenCV(
      const std::vector<TypeFrom,allocatorFrom>& vectFrom,
      std::vector<TypeTo,allocatorTo>& vectTo)
    {
      vectTo.clear();
      unsigned int it = 0,
        end = vectFrom.size();
      vectTo.reserve(end);

      while ( it < end )
      {
        vectTo.push_back( TypeTo() );//create an empty value
        float* datas = convert_to_float( vectFrom[it] );

        memcpy( reinterpret_cast<char*>( &vectTo[ it ] ),
          reinterpret_cast<const char*>( datas ),
          sizeof( TypeFrom ) );//assign the new value

        delete datas;
        it++;
      }
    }

    template<typename TypeFrom, typename TypeTo,
      typename allocatorFrom, typename allocatorTo>
    void convertVectorFrom_PCL(
      const std::vector<TypeFrom,allocatorFrom>& vectFrom,
      std::vector<TypeTo,allocatorTo>& vectTo)
    {
      vectTo.clear();
      unsigned int it = 0,
        end = vectFrom.size();
      vectTo.reserve(end);
      int sizeOfData = sizeof(TypeFrom) / sizeof( float );

      while ( it < end )
      {
        vectTo.push_back( TypeTo() );//create an empty value
        const float* datas = reinterpret_cast<const float*>( &vectFrom[it] );
        void *valConverted;
        int sizeOfType;

        //copy data using type:
        switch( TypeTo::depth )
        {
        case CV_8U:
          valConverted = convert_to_<uchar>( datas, sizeOfData );
          sizeOfType = sizeof(uchar);
          break;
        case CV_8S:
          valConverted = convert_to_<char>( datas, sizeOfData );
          sizeOfType = sizeof(char);
          break;
        case CV_16U:
          valConverted = convert_to_<ushort>( datas, sizeOfData );
          sizeOfType = sizeof(ushort);
          break;
        case CV_16S:
          valConverted = convert_to_<short>( datas, sizeOfData );
          sizeOfType = sizeof(short);
          break;
        case CV_32S:
          valConverted = convert_to_<int>( datas, sizeOfData );
          sizeOfType = sizeof(int);
          break;
        case CV_32F:
          valConverted = convert_to_<float>( datas, sizeOfData );
          sizeOfType = sizeof(float);
          break;
        case CV_64F:
          valConverted = convert_to_<double>( datas, sizeOfData );
          sizeOfType = sizeof(double);
          break;
        case CV_USRTYPE1:
          CV_Error( CV_StsBadFunc, "User type is not allowed!");
          break;
        }

        memcpy( reinterpret_cast<char*>( &vectTo[ it ] ),
          reinterpret_cast<const char*>( valConverted ),
          sizeof( TypeFrom ) );//assign the new value

        delete valConverted;
        it++;
      }
    }

    template<typename TypeFrom, typename allocatorFrom>
    void convertPCLVector(
      std::vector<TypeFrom,allocatorFrom>& vectFrom,
      cv::Mat& output, bool copyValues = false)
    {
      //PCL data are ALWAYS float, just the number of elements can change:
      int nbCol = sizeof(TypeFrom) / sizeof(float);
      output = cv::Mat( vectFrom.size(), nbCol,
        CV_32F, (void*)&vectFrom[0]);
      if( copyValues )
        output = output.clone();
    }
  }

}
#endif
