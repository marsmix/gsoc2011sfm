
#ifndef SFM_TEST_DATA_SETS_H_
#define SFM_TEST_DATA_SETS_H_

#include "../src/PointOfView.h"
#include "../src/CameraPinhole.h"
#include <opencv2/core/core.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

//Should not use "using namespace" but as we are in
//tutorial section, this will help readability
using namespace std;
using namespace OpencvSfM;
using namespace cv;

// Use this annotation at the end of a struct/class definition to
// prevent the compiler from optimizing away instances that are never
// used.
#if defined(__GNUC__) && !defined(COMPILER_ICC)
# define _ATTRIBUTE_UNUSED_ __attribute__ ((unused))
#else
# define _ATTRIBUTE_UNUSED_
#endif
// A macro to disallow operator=
// This should be used in the private: declarations for a class.
#define _DISALLOW_ASSIGN_(type)\
  void operator=(type const &)

// A macro to disallow copy constructor and operator=
// This should be used in the private: declarations for a class.
#define _DISALLOW_COPY_AND_ASSIGN_(type)\
  type(type const &);\
  _DISALLOW_ASSIGN_(type)

class Intern_tutorial_list;

// Defines the abstract factory interface that creates instances
// of a Test object.
class TutoFactoryBase {
 public:
  virtual ~TutoFactoryBase() {}

  // Creates a test instance to run. The instance is both created and destroyed
  // within TestInfoImpl::Run()
  virtual Intern_tutorial_list* CreateTest() = 0;

 protected:
  TutoFactoryBase(string n,string h) {name=n;help=h;}
  string name;
  string help;

 private:
  _DISALLOW_COPY_AND_ASSIGN_(TutoFactoryBase);
};

// This class create an instance of tutorial...
template <class TutoClass>
class TutoFactoryImpl : public TutoFactoryBase {
 public:
   TutoFactoryImpl(string n,string h):TutoFactoryBase(n,h){};
  virtual Intern_tutorial_list* CreateTest() {
    return new TutoClass(name,help); }
};

class Intern_tutorial_list
{
  static vector<Intern_tutorial_list*> list_of_tutos;
  _DISALLOW_COPY_AND_ASSIGN_(Intern_tutorial_list);
protected:
  Intern_tutorial_list(string name,string help)
  {
    this->name_of_tuto = name;
    this->tuto_help = help;
  }

  virtual void tuto_body()=0;

public:
  
  template <class TutoClass>
  static Intern_tutorial_list* registerTuto(TutoFactoryImpl<TutoClass>* addTuto)
  {
    Intern_tutorial_list* out = addTuto->CreateTest();
    list_of_tutos.push_back(out);
    return out;
  }

  static int print_menu()
  {
    cout<<"Please choose a tutorial (-1 to quit): "<<endl;
    for( unsigned int it = 0; it < list_of_tutos.size(); ++it)
    {
      cout<<it<<") "<<list_of_tutos[it]->name_of_tuto<<endl;
      cout<<"  "<<list_of_tutos[it]->tuto_help<<endl<<endl;
    }
    int rep=-1;
    cin>>rep;
    return rep;
  }

  static void run_tuto(int id_tuto)
  {
    if( id_tuto < 0 || id_tuto >= (int)list_of_tutos.size() )
      cout<<"wrong number, please try again..."<<endl;
    else
      list_of_tutos[id_tuto]->tuto_body();
  }

  string name_of_tuto;
  string tuto_help;
};

//////////////////////////////////////////////////////////////////////////
//Only to see how we can create a 3D structure estimation using calibrated cameras
//////////////////////////////////////////////////////////////////////////
enum { LOAD_INTRA=1, LOAD_POSITION=2, LOAD_FULL=3};

vector<PointOfView> loadCamerasFromFile(string fileName, int flag_model = LOAD_FULL);



#define TUTO_CLASS_NAME_(tuto_name) \
  class_##tuto_name##_Test
// Helper macro for defining tuto.
#define NEW_TUTO(t_name, tuto_name, tuto_help)\
class TUTO_CLASS_NAME_(t_name) : public Intern_tutorial_list {\
 public:\
  TUTO_CLASS_NAME_(t_name)(string n,string h)\
    :Intern_tutorial_list(n,h){}\
  static Intern_tutorial_list* const add_to_vector _ATTRIBUTE_UNUSED_;\
 private:\
  _DISALLOW_COPY_AND_ASSIGN_( TUTO_CLASS_NAME_(t_name) );\
  virtual void tuto_body();\
};\
Intern_tutorial_list* const TUTO_CLASS_NAME_(t_name)\
  ::add_to_vector = Intern_tutorial_list\
    ::registerTuto(new TutoFactoryImpl<TUTO_CLASS_NAME_(t_name)>(tuto_name, tuto_help));\
void TUTO_CLASS_NAME_(t_name)::tuto_body()



#endif  // LIBMV_MULTIVIEW_TEST_DATA_SETS_H_
