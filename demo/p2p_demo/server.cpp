/**
 * File: Demo.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DBoW2
 * License: see the LICENSE.txt file
 */

#include <cassert>
#include <iostream>
#include <vector>
#include <iomanip>
#include <string>
#include <thread>

// DBoW2
#include <DBoW2.h> // defines OrbVocabulary and OrbDatabase

// DLib
#include <DUtils/DUtils.h>
#include <DVision/DVision.h>

// OpenCV
#include <opencv2/opencv.hpp>

#include <zmq.h>

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

void testVocCreation(const vector<vector<cv::Mat> > &feats1);

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


void listener() {
  //  Socket to talk to clients
  void *context = zmq_ctx_new();
  void *responder = zmq_socket(context, ZMQ_REP);
  int rc = zmq_bind(responder, "tcp://*:5555");
  assert(rc == 0);

  while (1) {
    char buffer[10];
    zmq_recv(responder, buffer, 10, 0);
    printf("Server: Received Hello\n");
    sleep(1);          //  Do some 'work'
    zmq_send(responder, "World", 5, 0);
  }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void wait() {
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

int main(int argc, char* argv[]) {
  std::thread* ptListener = new thread(&listener);

  //Load ORB Vocabulary from disk (ORB-SLAM learnt vocabulary)
  cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
  std::string strVocFile = argv[2];
  ORBVocabulary* mpVocabulary = new ORBVocabulary();

  bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
  if (!bVocLoad) {
    cerr << "Wrong path to vocabulary. " << endl;
    cerr << "Falied to open at: " << strVocFile << endl;
    exit(-1);
  }
  cout << "Vocabulary loaded!" << endl << endl;
  cout << "Vocabulary information: " << endl << &mpVocabulary << endl << endl;

  /// Parse input parameters
  std::string videopath = argv[1];
  vector < vector<cv::Mat> > feats;
  loadFeatures(feats, videopath);

  testVocCreation(feats1);



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

void testVocCreation(const vector<vector<cv::Mat> > &feats1) {
  // lets do something with this vocabulary
  cout << "Matching images against themselves (0 low, 1 high): " << endl;

  BowVector v1;
  DBoW2::FeatureVector fv1;

  int nFeats1 = (int) feats1.size();

  for (int i = 0; i < nFeats1; i++) {
    cv::Mat D1;
    changeStructure(feats1[i], D1);
    mpVocabulary->transform(feats1[i], v1, fv1, 4);

    std::cout << "# features in first image: " << D1.rows << std::endl;
  }

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

