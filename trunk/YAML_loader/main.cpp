#include <string>
#include <iostream>
#include <sstream>
#include <numeric>
#include <algorithm>

#include <boost/filesystem.hpp>   // includes all needed Boost.Filesystem declarations

#include "../src/Visualizer.h"
#include "../src/SequenceAnalyzer.h"
#include "../src/StructureEstimator.h"

using namespace std;
using namespace OpencvSfM;
using namespace boost::filesystem;

//for comodity and easy reading, I put some global variables.
//Of course in a real application, you should avoid it!

vector< vector< unsigned int > > colors;
vector< vector< cv::Vec3d > > points3D;
vector< PointOfView > cameras;

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
  vector<char> mask =  structure.computeStructure(2);
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

  structure.removeOutliersTracks( 3 );//remove points whose reprojection err>3
  SequenceAnalyzer::keepOnlyCorrectMatches( struct_loaded, 3, 0 );

  cout<<struct_loaded.getTracks().size()<<" 3D points found."<<endl;

}

void loadPoints(string ymlFile)
{
  cv::FileStorage fsRead( ymlFile, cv::FileStorage::READ );
  cv::FileNode myFileNode = fsRead.getFirstTopLevelNode( );
  if( !myFileNode.empty() && myFileNode.name() == "SequenceAnalyzer" )
  {
    SequenceAnalyzer struct_loaded( myFileNode );
    cout<<"numbers of correct tracks loaded from "<<ymlFile<<":"<<
      struct_loaded.getTracks( ).size( )<<endl;

    vector< cv::Vec3d > tracks = struct_loaded.get3DStructure( );
    if( tracks.empty() )
    {
      triangul_tracks(cameras, struct_loaded);
      points3D.push_back( struct_loaded.get3DStructure( ) );
      colors.push_back( struct_loaded.getColors( ) );

      //now save the tracks:
      stringstream outName;
      outName<<ymlFile.substr(0,ymlFile.find_last_of('.'));
      outName<<"_good.yml";
      cv::FileStorage fsOutMotion1( outName.str(), cv::FileStorage::WRITE );
      SequenceAnalyzer::write( fsOutMotion1,struct_loaded );
      fsOutMotion1.release( );
    }
    else
    {
      vector< unsigned int > cols = struct_loaded.getColors( );
      points3D.push_back(tracks);
      colors.push_back(cols);
    }
  }
  fsRead.release();
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
    cerr<<"Usage is: "<<nameExe<<" cameras.yml directory"<<endl;
    exit(1);
  }

  cv::FileStorage fsRead( params[0], cv::FileStorage::READ );
  cv::FileNode myFileNode = fsRead.getFirstTopLevelNode( );
  if( !myFileNode.empty() )
  {
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
  }
  fsRead.release();
  if( cameras.empty() )
  {
    cerr<<"Please give as first parameter a camera yml.\n"
    "You can use for example the one in Medias/temple/cameras.yml"<<endl;
    exit(1);
  }

  //use boost to load each YAML files from directory:
  path dirTmp( params[1].c_str( ) );
  if ( !boost::filesystem::exists( dirTmp ) || !boost::filesystem::is_directory( dirTmp ) ) {
    cerr<<"Please give a correct folder name as second parameter"<<endl;
    exit(1);
  }

  directory_iterator iter= directory_iterator( dirTmp );
  while( iter != directory_iterator( ) )
  {
    string name_of_file = iter->path( ).string( );

    transform(name_of_file.begin(), name_of_file.end(),
      name_of_file.begin(),tolower);
    if( name_of_file.find(".yml")!=string::npos )
      loadPoints( name_of_file );

    iter++;
  }

  Visualizer my_viewer ( "3D Viewer" );

  for(size_t i=0; i<colors.size(); ++i)
  {
    stringstream nameOfCloud;
    nameOfCloud<<"Structure "<<i<<" triangulated";
    cout<<nameOfCloud.str()<<" : "<<points3D[i].size()<<endl;
    my_viewer.add3DPointsColored( points3D[i],colors[i], nameOfCloud.str() );
  }
  my_viewer.runInteract();
}