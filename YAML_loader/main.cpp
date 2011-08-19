#include <string>
#include <iostream>
#include <sstream>
#include <numeric>


#include "../src/Visualizer.h"
#include "../src/SequenceAnalyzer.h"
#include "../src/StructureEstimator.h"

using namespace std;
using namespace OpencvSfM;

vector<string> getParams(int argc, char *argv[])
{
  vector<string> out;
  for(int i=1;i<argc;++i)
    out.push_back( string(argv[i]) );
  return out;
}


void triangul_tracks(vector< PointOfView >& cameras,
  SequenceAnalyzer& struct_loaded)
{
  cout<<"triangulation of points."<<endl;
  StructureEstimator structure ( &struct_loaded, &cameras );
  vector<char> mask =  structure.computeStructure(5);
  vector< TrackOfPoints >& tracks = struct_loaded.getTracks();
  //remove bad tracks:
  int nbGoodTrack = 0;
  for(unsigned int d = 0, d_idx=0;d<mask.size(); d++,d_idx++)
    if(mask[d]==0)
    {
      //remove this bad match:
      tracks[d_idx] = tracks[tracks.size()-1];
      d_idx--;
      tracks.pop_back();
    }
    else
      nbGoodTrack++;

  structure.removeOutliersTracks( 2 );//remove points whose reprojection err>2

  cout<<nbGoodTrack<<" 3D points found."<<endl;
}

int main(int argc, char *argv[])
{
  vector<string> params = getParams( argc, argv );
  if( params.empty() )
  {
    string nameExe = argv[0];
    if(nameExe.find('\\')!=string::npos)
      nameExe = nameExe.substr(nameExe.find_last_of('\\')+1);
    if(nameExe.find('/')!=string::npos)
      nameExe = nameExe.substr(nameExe.find_last_of('/')+1);
    cerr<<"Usage is: "<<nameExe<<" filename1.yml [filenameX.yml]"<<endl;
    exit(1);
  }
  vector< vector< unsigned int > > colors;
  vector< vector< cv::Vec3d > > points3D;
  vector< PointOfView > cameras;

  for(int i=0; i<params.size(); ++i)
  {
    cv::FileStorage fsRead( params[i], cv::FileStorage::READ );
    cv::FileNode myFileNode = fsRead.getFirstTopLevelNode( );
    if( myFileNode.name() == "SequenceAnalyzer" )
    {
      vector< cv::Mat > images;//empty list of image as we don't need them here...
      SequenceAnalyzer struct_loaded( images, myFileNode );
      cout<<"numbers of correct tracks loaded:"<<
        struct_loaded.getTracks( ).size( )<<endl;

      vector< cv::Vec3d > tracks = struct_loaded.get3DStructure( );
      if( tracks.empty() )
      {
        if( cameras.empty() )
          cerr<<"The loaded tracks from "<<params[i]<<" don't have 3D coordinates... "
          "Please triangulate them before!"<<endl;
        else
        {
          triangul_tracks(cameras, struct_loaded);
          points3D.push_back( struct_loaded.get3DStructure( ) );
          colors.push_back( struct_loaded.getColors( ) );
        }
      }
      else
      {
        vector< unsigned int > cols = struct_loaded.getColors( );
        points3D.push_back(tracks);
        colors.push_back(cols);
      }
    }
    if( myFileNode.name() == "vector_PointOfView" )
    {
      int nbCameras = myFileNode.size();
      for(int i=0;i<nbCameras; i++)
      {
        cv::FileNode fileTmp = myFileNode[i]["PointOfView"];
        if(!fileTmp.empty())
          cameras.push_back( *PointOfView::read(fileTmp) );
      }
    }
    fsRead.release();
  }

  Visualizer my_viewer ( "3D Viewer" );

  for(int i=0; i<colors.size(); ++i)
  {
    stringstream nameOfCloud;
    nameOfCloud<<"Structure "<<i<<" triangulated";
    cout<<nameOfCloud.str()<<" : "<<points3D[i].size()<<endl;
    my_viewer.add3DPointsColored( points3D[i],colors[i], nameOfCloud.str() );
  }
  my_viewer.runInteract();
}