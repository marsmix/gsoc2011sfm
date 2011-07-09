

#include "test_data_sets.h"
vector<Intern_tutorial_list*> Intern_tutorial_list::list_of_tutos;

vector<PointOfView> loadCamerasFromFile(string fileName, int flag_model )
{
  vector<PointOfView> outVect;
  ifstream pointsDef(fileName);
  bool isOK=pointsDef.is_open();
  //first get the numbers of cameras:
  int nbCameras;
  if(pointsDef>>nbCameras)
  {
    string name_of_picture;
    Mat intra_params,rotation;
    Vec3d translation;
    intra_params.create(3, 3, CV_64F);
    rotation.create(3, 3, CV_64F);
    double* data_intra_param=(double*)intra_params.data;
    double* data_rotation=(double*)rotation.data;
    for (int i=0;i<nbCameras;i++)
    {
      //first the name of image:
      if(pointsDef>>name_of_picture)
      {
        //the 9 values of K:
        for(int j=0;j<9;j++)
          pointsDef>>data_intra_param[j];
        //the 9 values of rotation:
        for(int j=0;j<9;j++)
          pointsDef>>data_rotation[j];
        //the 3 values of translation:
        for(int j=0;j<3;j++)
          pointsDef>>translation[j];
        //now create a point of view:
        if( (flag_model & LOAD_FULL) == LOAD_FULL)
        {
          outVect.push_back(
            PointOfView(new CameraPinhole(intra_params),rotation,translation));
        }
        else
        {
          if( (flag_model & LOAD_FULL) == LOAD_POSITION)
          {
            outVect.push_back(
              PointOfView(new CameraPinhole(),rotation,translation));
          }
          else
          {
            if( (flag_model & LOAD_FULL) == LOAD_INTRA)
            {
              outVect.push_back(
                PointOfView(new CameraPinhole(intra_params)));
            }
          }
        }
      }
    }
  }
  return outVect;
}


int main()
{
  int choice = Intern_tutorial_list::print_menu();
  while( choice>=0 )
  {
    Intern_tutorial_list::run_tuto(choice);
    choice = Intern_tutorial_list::print_menu();
  }
}