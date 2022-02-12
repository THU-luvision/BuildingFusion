//#include "header.h"
#include "ServerMild.h"
namespace Server
{
//static ServerMild server_mild;
void ServerMild::select_closure_candidates(ServerFrame & f, std::vector<int> &candidate_frame_index,std::vector<int> &candidate_frame_submap,std::vector<int> &candidate_frame_camera)
{

  MILD::LoopClosureDetector & lcd = mild;
  MILD::BayesianFilter spatial_filter;
  std::vector<float > similarity_score;
  std::vector<int > &index = lcd.index;
  std::vector<int > &submapID = lcd.submapID;
  std::vector<int > &cameraID = lcd.cameraID;

  if(lcd.getDatabaseSize())
  {
  lcd.query_database(f.descriptor, similarity_score);
  candidate_frame_index.clear();
  std::vector<float> salient_score;
  std::vector<MILD::LCDCandidate> candidates;
  spatial_filter.calculateSalientScore(similarity_score, salient_score);
  bool allInitialized = true;
  if(!allInitialized)
  {
  std::vector<MILD::LCDCandidate> colli_candidates;


  //only select top 5, disgard the last frame
  for (int k = 0; k < lcd.getDatabaseSize()-1; k++)
  {
    //std::cout<<k<<" "<<salient_score[k]<<" "<<kflist[k].cameraID<<" "<<f.cameraID<<std::endl;
    if (salient_score[k] > salientScoreThreshold)
    {

      MILD::LCDCandidate candidate(salient_score[k],index[k],submapID[k],cameraID[k]);
      if(cameraID[k] == f.cameraID)
      candidates.push_back(candidate);
      else
      colli_candidates.push_back(candidate);
    }
  }

  std::sort(candidates.begin(), candidates.end(),greater<MILD::LCDCandidate>());
  std::sort(colli_candidates.begin(),colli_candidates.end(),greater<MILD::LCDCandidate>());
  for (int k = 0; k < fmin(candidates.size(), maxCandidateNum); k++)
  {
//    std::cout << kflist[candidates[k].index].keyFrameIndex << " " << candidates[k].salient_score << std::endl;
      candidate_frame_index.push_back(candidates[k].index);
      candidate_frame_submap.push_back(candidates[k].submapID);
      candidate_frame_camera.push_back(candidates[k].cameraID);
  }
  for (int k = 0; k < fmin(colli_candidates.size(), maxColliCandidateNum); k++)
  {
//    std::cout << kflist[candidates[k].index].keyFrameIndex << " " << candidates[k].salient_score << std::endl;
      candidate_frame_index.push_back(colli_candidates[k].index);
      candidate_frame_submap.push_back(colli_candidates[k].submapID);
      candidate_frame_camera.push_back(colli_candidates[k].cameraID);
  }
  }
  else
  {

      //only select top 5, disgard the last frame
      for (int k = 0; k < lcd.getDatabaseSize()-1; k++)
      {

        if (salient_score[k] > salientScoreThreshold)
        {

        MILD::LCDCandidate candidate(salient_score[k],index[k],submapID[k],cameraID[k]);
        candidates.push_back(candidate);
        }
      }
      std::sort(candidates.begin(), candidates.end(),greater<MILD::LCDCandidate>());
      for (int k = 0; k < fmin(candidates.size(), maxCandidateNum); k++)
      {
    //    std::cout << kflist[candidates[k].index].keyFrameIndex << " " << candidates[k].salient_score << std::endl;
      candidate_frame_index.push_back(candidates[k].index);
      candidate_frame_submap.push_back(candidates[k].submapID);
      candidate_frame_camera.push_back(candidates[k].cameraID);
      }
  }
  }
  lcd.construct_database(f.descriptor,f.frame_index,f.submapID,f.cameraID);
}
}

