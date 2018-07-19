/**
 * File: Demo.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DBoW2
 * License: see the LICENSE.txt file
 */

// STD Library
#include <map>
#include <iostream>
#include <vector>
#include <string>

// DBoW2
#include "DBoW2.h" // defines Surf64Vocabulary and Surf64Database

#include "FORB.h"
#include "TemplatedVocabulary.h"

#include <DUtils/DUtils.h>
#include <DVision/DVision.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d.hpp>

//using namespace DBoW2;
using namespace DUtils;
using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
  ORBVocabulary;

/**
 *
 */
static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

/**
 *
 */
void queryDatabase(std::map<int, DBoW2::BowVector> bowVecDB,
    DBoW2::BowVector queryBowVec, ORBVocabulary* mpVocabulary,
    float minScore, cv::Mat &scores, std::pair<int,float> &best);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeaturesPerImage(vector<vector<float> >  &features, const string imageName);
void loadFeatures(vector<vector<vector<float> > > &features);

void testDatabase(const vector<vector<vector<float> > > &features);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
/**
 *
 * @brief Read an ORB-SLAM message.
 * 
 * Read an ORB-SLAM message that contains the frame ID (key-frame) and
 * the number of points. For each point, the message stores the 3D world
 * position of the map point, the ORB descriptor associated to the
 * key-point observed in the current frame from the 3D point, and the
 * 2D position of the key-point in the image.
 *
 * @param filename The name of the message of camera N and frame F.
 * @param KeyPointsDesc An output Mx32 matrix in OpenCV format containing
 *          the ORB descriptor for each point in each row.
 *
 * @return frameID The index of the frame (key-frame).
 */
int readMessage(std::string filename, cv::OutputArray KeyPointsDesc);

/**
 *
 */
void readListKey(std::string keyfilesList,
		std::vector<std::string> &filenames);
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// number of training images
const int NIMAGES = 3;

// extended surf gives 128-dimensional vectors
const bool EXTENDED_SURF = false;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // Variable declarations
    int KF1id, KF2id, F1, F2;
    std::vector<std::string> keyfiles1, keyfiles2;

    float minScore = atof(argv[4]);
    
    //Load ORB Vocabulary trained offline
	cout << endl << "Loading ORB Vocabulary. This could take a while..."
			<< endl;

    string strVocFile = argv[1];

	ORBVocabulary* mpVocabulary = new ORBVocabulary();

    
    
	bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
	if (!bVocLoad) {
		cerr << "Wrong path to vocabulary. " << endl;
		cerr << "Falied to open at: " << strVocFile << endl;
		exit(-1);
	}
	cout << "Vocabulary loaded!" << endl << endl;

    // Read files with list of messages filenames
    readListKey(argv[2], keyfiles1);
    F1 = keyfiles1.size();

    // Create database
    // Read message from list of images af agent1
    cv::Mat mDescriptors, qDescriptors;
    
    std::map<int, DBoW2::BowVector> bowVecDB;
    std::map<int, DBoW2::FeatureVector> featVecDB;

    std::vector<int> lKF1id, lKF2id;


    for (int j=0; j < F1; ++j)
    {
        mDescriptors.release();

        KF1id = readMessage(keyfiles1[j], mDescriptors);

        std::cout << "Keyframe ID: " << KF1id << std::endl;
        
        if (KF1id == -1)
            continue;

        // Bag of Words Vector structures.
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;

        // Compute BoW for each message
        vector<cv::Mat> vCurrentDesc = toDescriptorVector(mDescriptors);

        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpVocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);

        bowVecDB[KF1id] = mBowVec;
        featVecDB[KF1id] = mFeatVec;

        lKF1id.push_back(KF1id);
    }

    // Read files with list of messages filenames
    readListKey(argv[3], keyfiles2);
    F2 = keyfiles2.size();

    cv::Mat S(F2,F1, CV_32FC1);
    std::map< int, std::pair<int, float> > overlap;
    
    for (int j=0; j < F2; ++j)
    {
        qDescriptors.release();

        // Query keyframe
        KF2id = readMessage(keyfiles2[j], qDescriptors);

        std::cout << "Query keyframe ID: " << KF2id << std::endl;
            
        if (KF2id == -1)
            continue;

        // Bag of Words Vector structures.
        DBoW2::BowVector queryBowVec;
        DBoW2::FeatureVector qFeatVec;

        // Compute BoW for each message
        vector<cv::Mat> vCurrentDesc = toDescriptorVector(qDescriptors);

        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpVocabulary->transform(vCurrentDesc,queryBowVec,qFeatVec,4);


        cv::Mat tmp;
        std::pair<int, float> bestKF;
        // Query database with another key-frame
        queryDatabase(bowVecDB, queryBowVec, mpVocabulary, minScore,
            tmp, bestKF);

        tmp = tmp.t();        
        tmp.copyTo(S.row(j));

        overlap[KF2id] = bestKF;

        lKF2id.push_back(KF2id);
    }

    //std::cout << S << std::endl;

    std::map<int, std::pair<int,float> >::iterator iti, ite;
    iti = overlap.begin();
    ite = overlap.end();
    
    std::pair<int, float> ps;
    for(; iti != ite; ++iti)
    {
        ps = iti->second;
        std::cout << "(" << iti->first << "," << ps.first <<
            ") = " << ps.second << std::endl;
    }

    // Save "overlap-based confusion matrix"
    FILE *fout;
    if ((fout = fopen("overlaps.txt", "w")) == NULL)
    {
		std::cout << "Could not open overlaps.txt for writing."
				<< std::endl;
		return false;
	}

    int N = S.rows; 
    int M = S.cols;
        
    for (int n = 0; n < N; ++n)
    {
        for (int m = 0; m < M; ++m)
        {
            fprintf(fout, "%.6f ", S.at<float>(n, m));
        }

        fprintf(fout, "\n");
    }

	fclose(fout);
    
    //std::cout << "Query the database.." << std::endl;
    
    //float si;
    //std::list< std::pair<float, int> > lScoreAndMatch;

    //std::map<int, DBoW2::BowVector>::iterator iti, ite, iti2, ite2;
    //iti = bowVecDB.begin();
    //ite = bowVecDB.end();
    
    //for (; iti != ite; ++iti)
    //{
        ////std::cout << iti->first << std::endl;
        //iti2 = bowVecDB.begin();
        //ite2 = bowVecDB.end();
    
        //for (; iti2 != ite2; ++iti2)
        //{
            ////if ( iti == iti2 )
            ////    continue;
            
            //si = mpVocabulary->score(iti->second, iti2->second);

            //if(si>=minScore)
                //lScoreAndMatch.push_back(make_pair(si,iti->first));

            //std::cout << iti->first << "," << iti2->first << "," << si << std::endl;
        //}
    //}

    

  return EXIT_SUCCESS;
}

// ----------------------------------------------------------------------------

//void loadFeaturesPerImage(vector<vector<float> >  &features, const string imageName)
//{
    //features.clear();
    //cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(400, 4, 2, EXTENDED_SURF);

    //cout << "Extracting SURF features..." << endl;
    //cv::Mat image = cv::imread(imageName, 0);
    //cv::Mat mask;
    //vector<cv::KeyPoint> keypoints;
    //vector<float> descriptors;

    //surf->detectAndCompute(image, mask, keypoints, descriptors);

    //vector<vector<vector<float> > > x;
    //x.push_back(vector<vector<float> >());
    //changeStructure(descriptors, x.back(), surf->descriptorSize());

    //features = x.back();
//}


//// ----------------------------------------------------------------------------

//void loadFeatures(vector<vector<vector<float> > > &features)
//{
  //features.clear();
  //features.reserve(NIMAGES);

  //cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(400, 4, 2, EXTENDED_SURF);

  //cout << "Extracting SURF features..." << endl;
  //for(int i = 0; i < NIMAGES; ++i)
  //{
    //stringstream ss;
    //ss << "images/image" << i << ".png";

    //cv::Mat image = cv::imread(ss.str(), 0);
    //cv::Mat mask;
    //vector<cv::KeyPoint> keypoints;
    //vector<float> descriptors;

    //surf->detectAndCompute(image, mask, keypoints, descriptors);

    //features.push_back(vector<vector<float> >());
    //changeStructure(descriptors, features.back(), surf->descriptorSize());
  //}
//}

//// ----------------------------------------------------------------------------

//void changeStructure(const vector<float> &plain, vector<vector<float> > &out,
  //int L)
//{
  //out.resize(plain.size() / L);

  //unsigned int j = 0;
  //for(unsigned int i = 0; i < plain.size(); i += L, ++j)
  //{
    //out[j].resize(L);
    //std::copy(plain.begin() + i, plain.begin() + i + L, out[j].begin());
  //}
//}

// ----------------------------------------------------------------------------

//void testVocCreation(const vector<vector<vector<float> > > &features, const string filename)
//{
  //// branching factor and depth levels 
  //const int k = 9;
  //const int L = 3;
  //const WeightingType weight = TF_IDF;
  //const ScoringType score = L1_NORM;

  //Surf64Vocabulary voc(k, L, weight, score);

  //cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
  //voc.create(features);
  //cout << "... done!" << endl;

  //cout << "Vocabulary information: " << endl
  //<< voc << endl << endl;

  //// lets do something with this vocabulary
  //cout << "Matching images against themselves (0 low, 1 high): " << endl;
  //BowVector v1, v2;
  //for(int i = 0; i < NIMAGES; i++)
  //{
    //voc.transform(features[i], v1);
    //for(int j = 0; j < NIMAGES; j++)
    //{
      //voc.transform(features[j], v2);
      
      //double score = voc.score(v1, v2);
      //cout << "Image " << i << " vs Image " << j << ": " << score << endl;
    //}
  //}

  //// save the vocabulary to disk
  //cout << endl << "Saving vocabulary..." << endl;
  //voc.save(filename);
  //cout << "Done" << endl;
//}

// ----------------------------------------------------------------------------

//void testDatabase(const vector<vector<vector<float> > > &features)
//{
  //cout << "Creating a small database..." << endl;

  //// load the vocabulary from disk
  //Surf64Vocabulary voc("small_voc.yml.gz");
  
  //Surf64Database db(voc, false, 0); // false = do not use direct index
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
  //Surf64Database db2("small_db.yml.gz");
  //cout << "... done! This is: " << endl << db2 << endl;
//}

// ----------------------------------------------------------------------------

// Read an ORB-SLAM message.
int readMessage(std::string filename, cv::OutputArray KeyPointsDesc)
{
    // Open message
	std::ifstream fKeyPoints(filename.c_str());
	if (!fKeyPoints.is_open()) {
		std::string msg = "Error opening file" + filename;
		perror(msg.c_str());
		return -1;
	}

    std::cout << "Reading file " << filename << " ..." << std::endl;

    // Declare variable to store each read line
	std::string line;

	getline(fKeyPoints, line);
	std::istringstream in(line);
	int nPoints, frameID;
	in >> frameID;
	in >> nPoints;

    
    //std::cout << "The number of 3D map points are " << nPoints << std::endl;

	//KeyPointsDesc.create(N, 32, CV_32SC1);
	cv::Mat D = KeyPointsDesc.getMat_();

    // Parse the file and store the descriptors
    //  - 3D point position
    //  - Keypoint descriptor in the current key-frame
    //  - Keypoint image position
	while (getline(fKeyPoints, line))
    {
        //std::cout << line << std::endl;

        if (line.empty())
            break;

        // Read descriptor line as the 3D position is read in the while condition
        getline(fKeyPoints, line);
        
		cv::Mat tmp(1, 32, CV_32SC1);
		std::istringstream in(line);

		for (int i = 0; i < 32; i += 4) {
			in >> tmp.at<int>(0, i);
			in >> tmp.at<int>(0, i + 1);
			in >> tmp.at<int>(0, i + 2);
			in >> tmp.at<int>(0, i + 3);
		}

		D.push_back(tmp);

        // Read 2D image points position line
		getline(fKeyPoints, line);
	}

	fKeyPoints.close();

    // Check that the number of descriptors corresponds to the number of
    // points declared at the beginning of the file
    //std::cout << "The number of 3D points read is: " << nPoints << std::endl;
    //std::cout << "The number of descriptors is: " << D.rows << std::endl;

    //if ((D.rows - 1) == nPoints)
    CV_Assert(nPoints == D.rows);

	D.convertTo(KeyPointsDesc, CV_8UC1);

    return frameID;
}

//
std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors)
{
    std::vector<cv::Mat> vDesc;
    vDesc.reserve(Descriptors.rows);
    for (int j=0;j<Descriptors.rows;j++)
        vDesc.push_back(Descriptors.row(j));

    return vDesc;
}

//
void readListKey(std::string keyfilesList,
		std::vector<std::string> &filenames) {

	std::ifstream keyfiles(keyfilesList.c_str());
	if (!keyfiles.is_open()) {
		std::string msg = "Error opening file" + keyfilesList;
		perror(msg.c_str());
		return;
	}

	std::string line;
	while (getline(keyfiles, line)) {
		filenames.push_back(line);
	}

	keyfiles.close();
}


//
void queryDatabase(std::map<int, DBoW2::BowVector> bowVecDB,
    DBoW2::BowVector queryBowVec, ORBVocabulary* mpVocabulary,
    float minScore, cv::Mat &scores, std::pair<int,float> &best)
{
    // Query database with another key-frame
    std::cout << "Query the database.." << std::endl;
    
    float si, best_si=0;
    std::list< std::pair<float, int> > lScoreAndMatch;

    int bestKFid=-1;

    std::map<int, DBoW2::BowVector>::iterator iti, ite, iti2, ite2;
    iti = bowVecDB.begin();
    ite = bowVecDB.end();
    
    for (; iti != ite; ++iti)
    {
        si = mpVocabulary->score(iti->second, queryBowVec);

        scores.push_back(si);

        //std::cout << "keyframe ID: " << iti->first << " - Score: " << si << std::endl;
        
        if(si>=minScore)
            lScoreAndMatch.push_back(make_pair(si,iti->first));

        if ( si > best_si)
        {
            best_si = si;
            bestKFid = iti->first;
        }
    }

    best = std::make_pair (bestKFid,best_si);
}

