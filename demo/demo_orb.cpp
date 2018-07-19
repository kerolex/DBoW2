/**
 * File: Demo.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DBoW2
 * License: see the LICENSE.txt file
 */

#include <iostream>
#include <vector>
#include <iomanip>
#include <string>

// DBoW2
#include "DBoW2.h" // defines OrbVocabulary and OrbDatabase

#include <DUtils/DUtils.h>
#include <DVision/DVision.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/imgproc/types_c.h>
#include <opencv2/videoio/videoio_c.h>


using namespace DBoW2;
using namespace DUtils;
using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeatures(vector<vector<cv::Mat > > &features, std::string
  videopath);
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out);
//void testVocCreation(const vector<vector<cv::Mat > > &features);
void testDatabase(const vector<vector<cv::Mat > > &features);

void testVocCreation(const vector<vector<cv::Mat > > &feats1, const
  vector<vector<cv::Mat > > &feats2, std::string strVocFile);


typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
  ORBVocabulary;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

int main(int argc, char* argv[])
{
  /// Parse input parameters
  std::string videopath1 = argv[1];
  std::string videopath2 = argv[2];
  std::string strVocFile = argv[3];

  vector<vector<cv::Mat > > feats1;
  loadFeatures(feats1, videopath1);

  vector<vector<cv::Mat > > feats2;
  loadFeatures(feats2, videopath2);

  testVocCreation(feats1, feats2, strVocFile);

  std::cout << "Finished!" << std::endl;
  
  return EXIT_SUCCESS;
}

// ----------------------------------------------------------------------------

void loadFeatures(vector<vector<cv::Mat > > &features, std::string
  videopath)
{
  cv::VideoCapture cam1(videopath + "%06d.png");

  if (!cam1.isOpened()) { return;}

  int nFrames = cam1.get(CV_CAP_PROP_FRAME_COUNT);
    
  features.clear();
  features.reserve(nFrames);

  cv::Ptr<cv::ORB> orb = cv::ORB::create();

  cout << "Extracting ORB features..." << endl;
  for(int i = 0; i < nFrames; ++i)
  {
    stringstream ss;
    ss << "images/image" << i << ".png";

    cv::Mat rgb_img;
    cam1 >> rgb_img; // Get a new frame from the video
    
    if ( rgb_img.empty() )
    {
        std::cerr << "!!! Failed imread(): image not found !!!" << std::endl;
        return;
    }


    // Convert RGB to gray image
    cv::Mat mImGray = rgb_img;
    if (mImGray.channels() == 3)
      { cvtColor(mImGray, mImGray, CV_RGB2GRAY); }

    cv::Mat mask;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    orb->detectAndCompute(mImGray, mask, keypoints, descriptors);

    features.push_back(vector<cv::Mat >());
    changeStructure(descriptors, features.back());
  }
}

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out)
{
  out.resize(plain.rows);

  for(int i = 0; i < plain.rows; ++i)
  {
    out[i] = plain.row(i);
  }
}

// ----------------------------------------------------------------------------

void testVocCreation(const vector<vector<cv::Mat > > &feats1, const
  vector<vector<cv::Mat > > &feats2, std::string strVocFile)
{
  // branching factor and depth levels 
  //const int k = 9;
  //const int L = 3;
  //const WeightingType weight = TF_IDF;
  //const ScoringType score = L1_NORM;

  //OrbVocabulary voc(k, L, weight, score);

  //cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
  //voc.create(features);
  //cout << "... done!" << endl;

  //cout << "Vocabulary information: " << endl
  //<< voc << endl << endl;

  //Load ORB Vocabulary from disk (ORB-SLAM learnt vocabulary)
  cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

  ORBVocabulary* mpVocabulary = new ORBVocabulary();
  
  bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
  if(!bVocLoad)
  {
      cerr << "Wrong path to vocabulary. " << endl;
      cerr << "Falied to open at: " << strVocFile << endl;
      exit(-1);
  }
  cout << "Vocabulary loaded!" << endl << endl;

  cout << "Vocabulary information: " << endl << &mpVocabulary << endl << endl;

  // lets do something with this vocabulary
  cout << "Matching images against themselves (0 low, 1 high): " << endl;
  BowVector v1, v2;

  std::string filename = "scores.dat";
	std::ofstream fPoses(filename.c_str());
	if (!fPoses.is_open())
  {
		std::string msg = "Error opening file" + filename;
		perror(msg.c_str());
		return;
	}

  int nFeats1 = (int)feats1.size();
  int nFeats2 = (int)feats2.size();
  
  for(int i = 0; i < nFeats1; i++)
  {
    mpVocabulary->transform(feats1[i], v1);
    for(int j = 0; j < nFeats2; j++)
    {
      mpVocabulary->transform(feats2[j], v2);
      
      double score = mpVocabulary->score(v1, v2);
      cout << "Image " << i << " vs Image " << j << ": " << score << endl;

      fPoses << std::setprecision(5) << std::setw(5)
				<< score << "\t";
    }
    fPoses << "\n";
  }

    fPoses.close();

  // save the vocabulary to disk
  //cout << endl << "Saving vocabulary..." << endl;
  //voc.save("small_voc.yml.gz");
  cout << "Done" << endl;
}

// ----------------------------------------------------------------------------

//void testDatabase(const vector<vector<cv::Mat > > &features,
  //std::string strVocFile)
//{
  //cout << "Creating a small database..." << endl;

  ////Load ORB Vocabulary from disk (ORB-SLAM learnt vocabulary)
  //cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

  //ORBVocabulary* mpVocabulary = new ORBVocabulary();
  
  //bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
  //if(!bVocLoad)
  //{
      //cerr << "Wrong path to vocabulary. " << endl;
      //cerr << "Falied to open at: " << strVocFile << endl;
      //exit(-1);
  //}
  //cout << "Vocabulary loaded!" << endl << endl;

  
  //OrbDatabase db(&mpVocabulary, false, 0); // false = do not use direct index
  //// (so ignore the last param)
  //// The direct index is useful if we want to retrieve the features that 
  //// belong to some vocabulary node.
  //// db creates a copy of the vocabulary, we may get rid of "voc" now

  //// add images to the database
  //for(int i = 0; i < NIMAGES; i++)
  //{
    //db.add(features[i]);
  //}

  //cout << "... done!" << endl;

  //cout << "Database information: " << endl << db << endl;

  //// and query the database
  //cout << "Querying the database: " << endl;

  //QueryResults ret;
  //for(int i = 0; i < NIMAGES; i++)
  //{
    //db.query(features[i], ret, 4);

    //// ret[0] is always the same image in this case, because we added it to the 
    //// database. ret[1] is the second best match.

    //cout << "Searching for Image " << i << ". " << ret << endl;
  //}

  //cout << endl;

  //// we can save the database. The created file includes the vocabulary
  //// and the entries added
  //cout << "Saving database..." << endl;
  //db.save("small_db.yml.gz");
  //cout << "... done!" << endl;
  
  //// once saved, we can load it again  
  //cout << "Retrieving database once again..." << endl;
  //OrbDatabase db2("small_db.yml.gz");
  //cout << "... done! This is: " << endl << db2 << endl;
//}

// ----------------------------------------------------------------------------


