#include "test_data_sets.h"
DECLARE_MUTEX( my_mutex_Tutorial_Handler );

vector<PointOfView> loadCamerasFromFile(string fileName, int flag_model )
{
  vector<PointOfView> outVect;
  ifstream pointsDef(fileName.c_str());
  bool isOK=pointsDef.is_open();
  if ( !isOK ){
    cout<<"Can't find "<<fileName<<"!\nPlease download the temple"
      "dataset from http://vision.middlebury.edu/mview/data/"<<endl;
  }
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


  int Tutorial_Handler::print_menu()
  {
    cout<<endl<<"//////////////////////////////////////////////////"<<endl;
    cout<<"Please choose a tutorial (-1 to quit): "<<endl;
    vector<Tutorial_Handler*> local_list_of_tutos =
      Intern_tutorial_list::getInstance()->list_of_tutos;
    for( unsigned int it = 0; it < local_list_of_tutos.size(); ++it)
    {
      cout<<it<<") "<<local_list_of_tutos[it]->name_of_tuto<<endl;
      cout<<"  "<<local_list_of_tutos[it]->tuto_help<<endl<<endl;
    }
    int rep=-1;
    cin>>rep;
    cin.clear(); //clear the error bits for the cin input stream
    cin.sync(); //synchronize the input buffer, discarding any leftover characters in the buffer 
    return rep;
  }

  bool Tutorial_Handler::run_tuto(int id_tuto)
  {
    vector<Tutorial_Handler*> local_list_of_tutos =
      Intern_tutorial_list::getInstance()->list_of_tutos;
    bool run_ok = false;
    if( id_tuto < 0 || id_tuto >= (int)local_list_of_tutos.size() )
      cout<<"wrong number, please try again..."<<endl;
    else
    {
      cout<<"Running "<<local_list_of_tutos[id_tuto]->file_of_tuto<<"..."<<endl;
      try{
        local_list_of_tutos[id_tuto]->tuto_body();
        run_ok = true;
      }catch(cv::Exception& e )
      {
        cout<<"Tutorial error..."<<endl;
      }
    }
    cout<<endl<<"Please type enter to continue"<<endl;
    std::cin.ignore( std::numeric_limits <std::streamsize> ::max(), '\n' );
    return run_ok;
  }

  bool Tutorial_Handler::ask_to_run_tuto(string name_of_tuto)
  {
    vector<Tutorial_Handler*> local_list =
      Intern_tutorial_list::getInstance()->list_of_tutos;
    //first get the id of tuto:

    bool run_ok = false;
    unsigned int id_of_tuto = -1;
    for(unsigned int it = 0; it < local_list.size(); ++it )
    {
      if( local_list[it]->id_of_tuto == name_of_tuto)
        id_of_tuto = it;
    }
    if( id_of_tuto<0 )
      return false;
    else
    {
      cout<<"Do you want to run "<<local_list[id_of_tuto]->name_of_tuto;
      cout<<"? (Y/n)"<<endl;
      char answer;
      cin>>answer;
      if( answer!='Y' && answer!='y' )
        return false;
      return run_tuto( id_of_tuto );
    }
  }


