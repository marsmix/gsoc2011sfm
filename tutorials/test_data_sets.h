
#ifndef _GSOC_SFM_TUTORIALS_H
#define _GSOC_SFM_TUTORIALS_H

#include "config.h"
#include "../src/PointOfView.h"
#include "../src/CameraPinhole.h"
#include <opencv2/core/core.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

// Use this annotation at the end of a struct/class definition to
// prevent the compiler from optimizing away instances that are never
// used.
#ifndef _ATTRIBUTE_UNUSED_
#if defined( __GNUC__ ) && !defined( COMPILER_ICC )
# define _ATTRIBUTE_UNUSED_ __attribute__ ( (unused ))
#else
# define _ATTRIBUTE_UNUSED_
#endif
#endif

namespace OpencvSfM{
  namespace tutorials{

    class Tutorial_Handler;

    class Intern_tutorial_list
    {
      std::vector<Tutorial_Handler*> list_of_tutos;
    public:
      static Intern_tutorial_list *getInstance( )
      {
        static Intern_tutorial_list *ref_static = 
          new Intern_tutorial_list( );
        return ref_static;
      }
      friend class Tutorial_Handler;
    };

    // This class create an instance of tutorial...
    template <class TutoClass>
    class TutoFactoryImpl {
    protected:
      std::string id_tuto;
      std::string name;
      std::string help;
      std::string file;
    public:
      TutoFactoryImpl(
        std::string id, std::string n, std::string h, std::string f )
      { id_tuto=id; name=n; help=h; file=f; };
      Tutorial_Handler* CreateTest( ) {
        return new TutoClass( id_tuto,name,help,file ); }
    };

    CREATE_EXTERN_MUTEX( my_mutex_Tutorial_Handler );

    class Tutorial_Handler
    {
    protected:
      Tutorial_Handler( std::string id, std::string name,
        std::string help, std::string file )
      {
        this->id_of_tuto = id;
        this->name_of_tuto = name;
        this->tuto_help = help;
        this->file_of_tuto = file;
      }

      virtual void tuto_body( )=0;

    public:

      template <class TutoClass>
      static Tutorial_Handler* registerTuto( TutoFactoryImpl<TutoClass>* addTuto )
      {
        Tutorial_Handler* out = addTuto->CreateTest( );
        P_MUTEX( my_mutex_Tutorial_Handler );
        vector<Tutorial_Handler*> &local_list_of_tutos =
          Intern_tutorial_list::getInstance( )->list_of_tutos;
        local_list_of_tutos.push_back( out );
        V_MUTEX( my_mutex_Tutorial_Handler );
        return out;
      }


      static int print_menu( );

      static bool run_tuto( int id_tuto );

      static bool ask_to_run_tuto( std::string name_of_tuto );

      std::string id_of_tuto;
      std::string name_of_tuto;
      std::string file_of_tuto;
      std::string tuto_help;
    };


    //////////////////////////////////////////////////////////////////////////
    //Only to see how we can create a 3D structure estimation using calibrated cameras
    //////////////////////////////////////////////////////////////////////////
    enum { LOAD_INTRA=1, LOAD_POSITION=2, LOAD_FULL=3};

    std::vector<PointOfView> loadCamerasFromFile( std::string fileName,
      int flag_model = LOAD_FULL );


#define TOSTR( x ) #x
#define TUTO_CLASS_NAME_( tuto_name )  class_##tuto_name##_Test

    // Helper macro for defining tuto.
#define NEW_TUTO( t_name, tuto_name, tuto_help )\
    class TUTO_CLASS_NAME_( t_name ) :\
    public Tutorial_Handler {\
    public:\
    TUTO_CLASS_NAME_( t_name )( std::string id, std::string n,\
    std::string h, std::string f ) : Tutorial_Handler( id,n,h,f ){}\
    static Tutorial_Handler* const add_to_vector _ATTRIBUTE_UNUSED_;\
    virtual void tuto_body( );\
    };\
    Tutorial_Handler* const TUTO_CLASS_NAME_( t_name )\
    ::add_to_vector = Tutorial_Handler\
    ::registerTuto( new TutoFactoryImpl<TUTO_CLASS_NAME_( t_name )>( \
    TOSTR( t_name ), tuto_name, tuto_help, __FILE__ ));\
    void TUTO_CLASS_NAME_( t_name )::tuto_body( )

  }
}


#endif  // _GSOC_SFM_TUTORIALS_H