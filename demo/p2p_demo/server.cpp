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
#include <opencv2/opencv.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/features2d.hpp>
//#include <opencv2/videoio.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/imgproc.hpp>

// Compatibility with OpenCV < 3.4
//#include <opencv2/imgproc/types_c.h>
//#include <opencv2/videoio/videoio_c.h>

using namespace DBoW2;
using namespace DUtils;
using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
const int DESC_TH = 50;
const float LOWE_RATIO = 0.8;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void loadFeatures(vector<vector<cv::Mat> > &features, std::string videopath);

/**
 *
 */
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out);

// @Override
void changeStructure(const vector<cv::Mat> &plain_vec, cv::Mat& out);


//void testVocCreation(const vector<vector<cv::Mat > > &features);
void testDatabase(const vector<vector<cv::Mat> > &features);

void testVocCreation(const vector<vector<cv::Mat> > &feats1,
                     const vector<vector<cv::Mat> > &feats2,
                     std::string strVocFile);

/**
 * @fn
 *
 * @brief
 *
 * Features are matched using the fast BoW approach and the nearest neighbour
 * with distance ratio to remove ambiguities and minimum threshold
 */
int SearchByBoW(const cv::Mat& desc1, const cv::Mat& desc2,
                const DBoW2::FeatureVector &vFeatVec1,
                const DBoW2::FeatureVector &vFeatVec2,
                std::map<int, int>& matches12, const int& desc_th,
                const float& lowe_ratio);

/**
 * @fn DescriptorDistance
 *
 * Bit set count operation from
 * http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
 *
 * @param a First ORB descriptor whose dimension is 32 bytes (i.e. 32 elements
 * of uchar values)
 * @param a Second ORB descriptor whose dimension is 32 bytes (i.e. 32 elements
 * of uchar values)
 */
int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);



typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void wait() {
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
  /// Parse input parameters
  std::string videopath1 = argv[1];
  std::string videopath2 = argv[2];
  std::string strVocFile = argv[3];

  vector < vector<cv::Mat> > feats1;
  loadFeatures(feats1, videopath1);

  vector < vector<cv::Mat> > feats2;
  loadFeatures(feats2, videopath2);

  testVocCreation(feats1, feats2, strVocFile);

  std::cout << "Finished!" << std::endl;

  return EXIT_SUCCESS;
}

// ----------------------------------------------------------------------------

void loadFeatures(vector<vector<cv::Mat> > &features, std::string videopath) {
  cv::VideoCapture cam1(videopath + "%06d.png");

  if (!cam1.isOpened()) {
    return;
  }

  int nFrames = cam1.get(CV_CAP_PROP_FRAME_COUNT);

  features.clear();
  features.reserve(nFrames);

  cv::Ptr < cv::ORB > orb = cv::ORB::create(2000);

  cout << "Extracting ORB features..." << endl;
  for (int i = 0; i < nFrames; ++i) {
    stringstream ss;
    ss << "images/image" << i << ".png";

    cv::Mat rgb_img;
    cam1 >> rgb_img;  // Get a new frame from the video

    if (rgb_img.empty()) {
      std::cerr << "!!! Failed imread(): image not found !!!" << std::endl;
      return;
    }

    // Convert RGB to gray image
    cv::Mat mImGray = rgb_img;
    if (mImGray.channels() == 3) {
      cvtColor(mImGray, mImGray, CV_RGB2GRAY);
    }

    cv::Mat mask;
    vector < cv::KeyPoint > keypoints;
    cv::Mat descriptors;

    orb->detectAndCompute(mImGray, mask, keypoints, descriptors);

    features.push_back(vector<cv::Mat>());
    changeStructure(descriptors, features.back());
  }
}

// ----------------------------------------------------------------------------

void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out) {
  out.resize(plain.rows);

  for (int i = 0; i < plain.rows; ++i) {
    out[i] = plain.row(i);
  }
}

void changeStructure(const vector<cv::Mat> &plain_vec, cv::Mat& out) {

  out.release();

  for (size_t i = 0; i < plain_vec.size(); ++i) {
    out.push_back(plain_vec[i].clone());
  }
}

// ----------------------------------------------------------------------------

void testVocCreation(const vector<vector<cv::Mat> > &feats1,
                     const vector<vector<cv::Mat> > &feats2,
                     std::string strVocFile) {
  //Load ORB Vocabulary from disk (ORB-SLAM learnt vocabulary)
  cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

  ORBVocabulary* mpVocabulary = new ORBVocabulary();

  bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
  if (!bVocLoad) {
    cerr << "Wrong path to vocabulary. " << endl;
    cerr << "Falied to open at: " << strVocFile << endl;
    exit(-1);
  }
  cout << "Vocabulary loaded!" << endl << endl;

  cout << "Vocabulary information: " << endl << &mpVocabulary << endl << endl;

  // lets do something with this vocabulary
  cout << "Matching images against themselves (0 low, 1 high): " << endl;

  BowVector v1, v2;
  DBoW2::FeatureVector fv1, fv2;

  std::map<int, int> matches12;

  std::string filename = "scores.dat";
  std::ofstream fPoses(filename.c_str());
  if (!fPoses.is_open()) {
    std::string msg = "Error opening file" + filename;
    perror(msg.c_str());
    return;
  }

  std::string filename2 = "matches.dat";
  std::ofstream fmatches(filename2.c_str());
  if (!fmatches.is_open()) {
    std::string msg = "Error opening file" + filename2;
    perror(msg.c_str());
    return;
  }
  
    std::string filename3 = "matches_normalised.dat";
  std::ofstream fmatchesnorm(filename3.c_str());
  if (!fmatchesnorm.is_open()) {
    std::string msg = "Error opening file" + filename3;
    perror(msg.c_str());
    return;
  }

  int nFeats1 = (int) feats1.size();
  int nFeats2 = (int) feats2.size();

  double best_score = 0;

  int idx1 = -1;
  int idx2 = -1;

  for (int i = 0; i < nFeats1; i++) {

    cv::Mat D1;
    changeStructure(feats1[i], D1);
    mpVocabulary->transform(feats1[i], v1, fv1, 4);

    for (int j = 0; j < nFeats2; j++) {

      matches12.clear();

      cv::Mat D2;
      changeStructure(feats2[j], D2);
      mpVocabulary->transform(feats2[j], v2, fv2, 4);

      std::cout << "# features in first image: " << D1.rows << std::endl;
      std::cout << "# features in second image: " << D2.rows << std::endl;

      double score = mpVocabulary->score(v1, v2);

      int num_matches = SearchByBoW(D1, D2, fv1, fv2, matches12, DESC_TH,
                                    LOWE_RATIO);

      cout << "Image " << i << " vs Image " << j << ": " << score
          << " ( # matches = " << num_matches << ")" << endl;

      fPoses << std::setprecision(5) << std::setw(5) << score << "\t";
      fmatches << std::setw(5) << num_matches << "\t";
            fmatchesnorm << std::setprecision(5) << std::setw(5) << num_matches / (float)min(D1.rows, D2.rows) << "\t";

      if (i != j && score > best_score) {
        idx1 = i;
        idx2 = j;
        best_score = score;
      }
    }
    fPoses << "\n";
    fmatches << "\n";
    fmatchesnorm << "\n";
  }

  fPoses.close();
  fmatches.close();
  fmatchesnorm.close();

  cv::Mat desc1, desc2;
  changeStructure(feats1[idx1], desc1);
  changeStructure(feats2[idx2], desc2);

  mpVocabulary->transform(feats1[idx1], v1, fv1, 4);
  mpVocabulary->transform(feats2[idx2], v2, fv2, 4);

  int num_matches = SearchByBoW(desc1, desc2, fv1, fv2, matches12, DESC_TH,
                                LOWE_RATIO);

  std::cout << "The number of matches between image #" << idx1 << " and image #"
      << idx2 << " (score = " << best_score << ") is " << num_matches
      << std::endl;

  // TODO: visualise matches

  cout << "Done" << endl;
}


// Features are matched using the fast BoW approach and the nearest neighbour
// with distance ratio to remove ambiguities and minimum threshold
int SearchByBoW(const cv::Mat& desc1, const cv::Mat& desc2,
                const DBoW2::FeatureVector &vFeatVec1,
                const DBoW2::FeatureVector &vFeatVec2,
                std::map<int, int>& matches12, const int& desc_th,
                const float& lowe_ratio) {

  matches12.clear();

  int nmatches = 0;  // number of matches

  DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
  DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
  DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
  DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

  while (f1it != f1end && f2it != f2end) {
    if (f1it->first == f2it->first) {
      for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
        const size_t idx1 = f1it->second[i1];

        const cv::Mat &d1 = desc1.row(idx1);

        int bestDist1 = 256;
        int bestIdx2 = -1;
        int bestDist2 = 256;

        for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
          const size_t idx2 = f2it->second[i2];

          const cv::Mat &d2 = desc2.row(idx2);

          int dist = DescriptorDistance(d1, d2);

          if (dist < bestDist1) {
            bestDist2 = bestDist1;
            bestDist1 = dist;
            bestIdx2 = idx2;
          } else if (dist < bestDist2) {
            bestDist2 = dist;
          }
        }

        if (bestDist1 < desc_th) {
          if (static_cast<float>(bestDist1)
              < lowe_ratio * static_cast<float>(bestDist2)) {
            matches12[idx1] = bestIdx2;
            nmatches++;
          }
        }
      }

      f1it++;
      f2it++;
    } else if (f1it->first < f2it->first) {
      f1it = vFeatVec1.lower_bound(f2it->first);
    } else {
      f2it = vFeatVec2.lower_bound(f1it->first);
    }
  }

  return nmatches;
}


//
int DescriptorDistance(const cv::Mat &a, const cv::Mat &b) {
  const int *pa = a.ptr<int32_t>();
  const int *pb = b.ptr<int32_t>();

  int dist = 0;

  for (int i = 0; i < 8; i++, pa++, pb++) {
    unsigned int v = *pa ^ *pb;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }

  return dist;
}

