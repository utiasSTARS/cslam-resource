/**
 * Author: Matthew Giamou, original example by Dorian Galvez-Lopez
 * Description: demo application of DBoW2
 * License: see the LICENSE.txt file
 */

#include <iostream>
#include <vector>
#include <iomanip>  // For setw and setfill
#include <numeric>

// DBoW2
#include "DBoW2.h" // defines OrbVocabulary and OrbDatabase

#include <DUtils/DUtils.h>
#include <DVision/DVision.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>


using namespace DBoW2;
using namespace DUtils;
using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeatures(vector<vector<cv::Mat > > &features);
void testVocCreation(const vector<vector<cv::Mat > > &features);

// number of training images from KITTI datasets
const vector<string> SEQUENCES = {"00", "02", "05", "06", "08"};
const vector<int> SEQUENCE_SIZES = {4541, 4661, 2761, 1101, 4071};
string KITTI_BASE_DIR = "/drive1/Datasets/kitti/odometry/dataset/sequences/";

int main()
{
  vector<vector<cv::Mat > > features;
  loadFeatures(features);
  testVocCreation(features);
  return 0;
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

void loadFeatures(vector<vector<cv::Mat > > &features)
{
  features.clear();
  features.reserve(accumulate(SEQUENCE_SIZES.begin(), SEQUENCE_SIZES.end(), 0));

  // ORB default FAST threshold=20, max features per image = 500
  cv::Ptr<cv::ORB> orb = cv::ORB::create(); 

  cout << "Extracting ORB features..." << endl;
  for (int i = 0; i < SEQUENCES.size(); ++i) {
    for (int j = 0; j < SEQUENCE_SIZES[i]; ++j) {
      stringstream ss;
      ss << KITTI_BASE_DIR << SEQUENCES[i] << "/image_2/" << setfill('0') << setw(6) << j << ".png";
      if (j%1000 == 0) {
        cout << "Filename: " << endl << ss.str() << endl;
      }
      cv::Mat image = cv::imread(ss.str(), 0);
      cv::Mat mask;
      vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;
      orb->detectAndCompute(image, mask, keypoints, descriptors);
      features.push_back(vector<cv::Mat >());
      changeStructure(descriptors, features.back());
    }
  }
}


// ----------------------------------------------------------------------------

void testVocCreation(const vector<vector<cv::Mat > > &features)
{
  // branching factor and depth levels 
  const int k = 10;
  const int L = 6;
  const WeightingType weight = TF_IDF;
  const ScoringType score = L1_NORM;

  OrbVocabulary voc(k, L, weight, score);

  cout << "Creating a " << k << "^" << L << " vocabulary with KITT odometry datasets..." << endl;
  voc.create(features);
  cout << "... done!" << endl;

  cout << "Vocabulary information: " << endl
  << voc << endl << endl;

  // lets do something with this vocabulary
  // cout << "Matching images against themselves (0 low, 1 high): " << endl;
  // BowVector v1, v2;
  // for(int i = 0; i < NIMAGES; i++)
  // {
  //   voc.transform(features[i], v1);
  //   for(int j = 0; j < NIMAGES; j++)
  //   {
  //     voc.transform(features[j], v2);
      
  //     double score = voc.score(v1, v2);
  //     cout << "Image " << i << " vs Image " << j << ": " << score << endl;
  //   }
  // }

  // save the vocabulary to disk
  cout << endl << "Saving vocabulary..." << endl;
  voc.save("KITTI_odometry.yml.gz");
  cout << "Done" << endl;
}


