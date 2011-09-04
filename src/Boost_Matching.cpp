#include "Boost_Matching.h"

#include "CameraPinholeDistor.h"
#include <iostream>

namespace OpencvSfM{

  using cv::Ptr;
  using cv::Mat;
  using std::vector;

  size_t MatchingThread::size_list;
  std::vector< cv::Ptr< PointsToTrack > >* MatchingThread::matches_ = NULL;
  vector< cv::Mat > MatchingThread::masks;
  unsigned int MatchingThread::mininum_points_matches=50;
  PointsMatcher* MatchingThread::match_algorithm = NULL;
  double MatchingThread::total_matches = 0;
  double MatchingThread::current_match_ = 0;
  bool MatchingThread::print_progress_ = true;

  DECLARE_MUTEX( MatchingThread::thread_concurr );
  DECLARE_MUTEX( MatchingThread::thread_unicity );


  MatchingThread::MatchingThread(cv::Ptr<  SequenceAnalyzer> seq_analyser,
    unsigned int i)
  {
    this->i = i;
    this->seq_analyser = seq_analyser;
    this->seq_analyser.addref();//avoid ptr deletion...
  }

  void MatchingThread::operator()()
  {
    Ptr<PointsToTrack> points_to_track_i=( *matches_ )[i];
    double error_allowed = MAX( seq_analyser->images_[ i ].rows,
      seq_analyser->images_[ i ].cols ) * 0.004;

    points_to_track_i->computeKeypointsAndDesc( false );

    P_MUTEX( thread_unicity );
    Ptr<PointsMatcher> point_matcher = match_algorithm->clone( true );
    point_matcher->add( points_to_track_i );
    V_MUTEX( thread_unicity );
    point_matcher->train( );

    unsigned int j=i+1;
    while ( j < size_list )
    {
      Ptr<PointsToTrack> points_to_track_j=( *matches_ )[j];

      points_to_track_j->computeKeypointsAndDesc( false );

      P_MUTEX( thread_unicity );
      current_match_++;
      if( print_progress_ )
      {
        if( ( ((int) ((current_match_*100)/total_matches) )%10 ) == 0 )
          if( ((int) current_match_) % ((int) (total_matches / 100) + 1 ) == 0)
            std::cout<<(int)((current_match_*100)/total_matches)<<" %"<<std::endl;
      }
      Ptr<PointsMatcher> point_matcher1 = match_algorithm->clone( true );
      point_matcher1->add( points_to_track_j );
      V_MUTEX( thread_unicity );
      point_matcher1->train( );

      vector< cv::DMatch > matches_i_j;
      point_matcher->crossMatch( point_matcher1, matches_i_j, masks );

      //First compute points matches:
      unsigned int size_match=matches_i_j.size( );
      vector<cv::Point2f> srcP;
      vector<cv::Point2f> destP;
      vector<uchar> status;

      if( size_match>8 )
      {
        std::clog<<"Using crossMatch, found "<<matches_i_j.size( )<<
          " matches between "<<i<<" "<<j<<std::endl;
        //vector<KeyPoint> points1 = point_matcher->;
        for( size_t cpt = 0; cpt < size_match; ++cpt )
        {
          const cv::DMatch& match = matches_i_j[ cpt ];
          const cv::KeyPoint &key1 = point_matcher1->getKeypoint(
            matches_i_j[ cpt ].queryIdx );
          const cv::KeyPoint &key2 = point_matcher->getKeypoint(
            matches_i_j[ cpt ].trainIdx );
          srcP.push_back( cv::Point2f( key1.pt.x,key1.pt.y ) );
          destP.push_back( cv::Point2f( key2.pt.x,key2.pt.y ) );
          status.push_back( 1 );
        }

        //free some memory:
        point_matcher1->clear();
        points_to_track_j->free_descriptors();

        Mat fundam = cv::findFundamentalMat( srcP, destP, status,
          cv::FM_RANSAC, error_allowed );

        unsigned int nbErrors = 0, nb_iter=0;
        //refine the mathing :
        size_match = status.size( );
        for( size_t cpt = 0; cpt < size_match; ++cpt )
        {
        if( status[ cpt ] == 0 )
        {
          size_match--;
          status[ cpt ] = status[ size_match ];
          status.pop_back( );
          srcP[ cpt ] = srcP[ size_match ];
          srcP.pop_back( );
          destP[ cpt ] = destP[ size_match ];
          destP.pop_back( );
          matches_i_j[ cpt ] = matches_i_j[ size_match ];
          matches_i_j.pop_back( );
          cpt--;
          ++nbErrors;
        }
      }

        while( nbErrors > size_match/10 && nb_iter < 3 &&
          matches_i_j.size( ) > mininum_points_matches )
        {
          fundam = cv::findFundamentalMat( srcP, destP, status,
            cv::FM_RANSAC, error_allowed*1.5 );

          //refine the mathing :
          nbErrors =0 ;
          size_match = status.size( );
          for( size_t cpt = 0; cpt < size_match; ++cpt ){
            if( status[ cpt ] == 0 )
            {
              size_match--;
              status[ cpt ] = status[ size_match ];
              status.pop_back( );
              srcP[ cpt ] = srcP[ size_match ];
              srcP.pop_back( );
              destP[ cpt ] = destP[ size_match ];
              destP.pop_back( );
              matches_i_j[ cpt ] = matches_i_j[ size_match ];
              matches_i_j.pop_back( );
              cpt--;
              ++nbErrors;
            }
          }
          nb_iter++;
        };

        //refine the mathing:
        fundam = cv::findFundamentalMat( srcP, destP, status, cv::FM_LMEDS );

        //refine the mathing :
        size_match = status.size( );
        for( size_t cpt = 0; cpt < size_match; ++cpt ){
          if( status[ cpt ] == 0 )
          {
            size_match--;
            status[ cpt ] = status[ size_match ];
            status.pop_back( );
            srcP[ cpt ] = srcP[ size_match ];
            srcP.pop_back( );
            destP[ cpt ] = destP[ size_match ];
            destP.pop_back( );
            matches_i_j[ cpt ] = matches_i_j[ size_match ];
            matches_i_j.pop_back( );
            cpt--;
            ++nbErrors;
          }
        }

        if( matches_i_j.size( ) > mininum_points_matches )
        {
          Mat * copy_of_fund = new cv::Mat();
          *copy_of_fund = fundam.clone();
          P_MUTEX( thread_unicity );
          seq_analyser->list_fundamental_[i][j-i] = cv::Ptr< cv::Mat >(
            copy_of_fund );
          seq_analyser->addMatches( matches_i_j,i,j );
          std::clog<<"; find "<<matches_i_j.size( )<<
            " real matches"<<std::endl;
          V_MUTEX( thread_unicity );
        }
        else
        {
          std::clog<<"Between "<<i<<" "<<j<<", can't find real matches"<<std::endl;
        }

      }
      else
      {
        std::clog<<"Using crossMatch, found only "<<matches_i_j.size( )<<
          " matches between "<<i<<" "<<j<<std::endl;
      }
      j++;
    }

    P_MUTEX( thread_unicity );
    point_matcher->clear();
    points_to_track_i->free_descriptors();//save memory...
    V_MUTEX( thread_unicity );
    V_MUTEX( MatchingThread::thread_concurr );//wake up waiting thread
  };
}
