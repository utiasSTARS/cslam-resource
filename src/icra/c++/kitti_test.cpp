/**
 * Author: Matthew Giamou, original example by Dorian Galvez-Lopez
 * Description: demo application of DBoW2
 * License: see the LICENSE.txt file
 */

#include <iostream>
#include <vector>
#include <iomanip>  // For setw and setfill

// DBoW2
#include "DBoW2.h" // defines OrbVocabulary and OrbDatabase

#include <DUtils/DUtils.h>
#include <DVision/DVision.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace DBoW2;
using namespace DUtils;
using namespace std;
using namespace Eigen;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeatures(vector<vector<cv::Mat > > &features, string sequence, int sequence_size);
void changeStructure(const cv::Mat &plain, vector<cv::Mat> &out);
void testDatabase(const vector<vector<cv::Mat > > &features, string sequence, int n_split, int n_best_matches);


// number of training images from KITTI datasets
// const vector<string> SEQUENCES = {"00", "02", "05", "06", "08"};
const vector<string> SEQUENCES = {"00","06"};
// const vector<int> SEQUENCE_SIZES = {4541, 4661, 2761, 1101, 4071};
const vector<int> SEQUENCE_SIZES = {4541, 1101};


// Hard code these from examining Python plots
// const vector<int> SEQUENCE_SPLITS = {(int)(4541.0/2.0), (int)(4661.0/2.0), 
//                                      (int)(2761.0/2.0), (int)(1101.0/2.0), (int)(4071.0/2.0)};
const vector<int> SEQUENCE_SPLITS = {(int)(4541.0/2.0), (int)(1101.0/2.0)};
string KITTI_BASE_DIR = "/drive1/Datasets/kitti/odometry/dataset/sequences/";

int fast_threshold = 100;
// int n_features_max = 500;
// This should be controlled by fast_threshold!!
int n_features_max = 100000;
int n_best_matches = 100;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// Define Eigen csv saving 
const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");

void wait()
{
  cout << endl << "Press enter to continue" << endl;
  getchar();
}

// ----------------------------------------------------------------------------

int main()
{
  for (int idx = 0; idx < SEQUENCES.size(); ++idx) {
    vector<vector<cv::Mat > > features;
    loadFeatures(features, SEQUENCES[idx], SEQUENCE_SIZES[idx]);
    cout << "Finished loading features." << endl;
    testDatabase(features, SEQUENCES[idx], SEQUENCE_SPLITS[idx], n_best_matches);
  }

  return 0;
}

// ----------------------------------------------------------------------------

void loadFeatures(vector<vector<cv::Mat > > &features, string sequence, int sequence_size)
{
  features.clear();
  features.reserve(sequence_size);

  // Make sure to set the params properly
  cv::Ptr<cv::ORB> orb = cv::ORB::create(n_features_max, 1.2f, 8, 31, 0, 2, 
                                         cv::ORB::HARRIS_SCORE, 31, fast_threshold);

  cout << "Extracting ORB features..." << endl;
  for(int i = 0; i < sequence_size; ++i)
  {
    stringstream ss;
    ss << KITTI_BASE_DIR << sequence << "/image_2/" << setfill('0') << setw(6) << i << ".png";

    if (i%1000 == 0) {
      cout << "File: " << ss.str() << endl;
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

void testDatabase(const vector<vector<cv::Mat > > &features, string sequence, int n_split, int n_best_matches)
{
  // load the vocabulary from disk
  OrbVocabulary voc("/home/mattgiamou/acl/scanexchange17/code/c++/DBoW2/vocabularies/KITTI_odometry_K10_L6.yml.gz");  
  OrbDatabase db(voc, false, 0); // false = do not use direct index

  cout << "Vocabulary loaded" << endl;
  // (so ignore the last param)
  // The direct index is useful if we want to retrieve the features that 
  // belong to some vocabulary node.
  // db creates a copy of the vocabulary, we may get rid of "voc" now

  // add images to the database
  for(int i = 0; i < n_split; i++)
  {
    db.add(features[i]);
  }
  cout << "Database of first half's information for correction: " << endl << db << endl;

  // and query the database
  cout << "Querying the database with second half for correction: " << endl;

  //Save output as a big matrix in a file, turn that into edges in Python.
  MatrixXi match_ids(features.size()-n_split, n_best_matches);
  MatrixXd match_scores(features.size()-n_split, n_best_matches);
  QueryResults ret; // This is a vector of Results, sorted from highest to lowest score
  for(int i = n_split; i < features.size(); ++i)
  {
    // Get previous frames similarity score for normalization
    BowVector current_frame;
    voc.transform(features[i], current_frame);
    BowVector previous_frame; 
    voc.transform(features[i-1], previous_frame);

    L1Scoring l1_scoring;
    double score_norm = l1_scoring.score(current_frame, previous_frame);

    if ( (i-n_split) % 1000 == 0) {
      cout << "Score norm " << i << ": " << score_norm << endl;
    }
    db.query(features[i], ret, n_best_matches);
    for (int j = 0; j < ret.size(); ++j) {
      match_ids(i-n_split, j) = ret[j].Id;
      match_scores(i-n_split, j) = ret[j].Score/score_norm;
    }
    // cout << "Searching for Image " << i << ". " << ret << endl;
  }

  stringstream score_string;
  score_string << "score_norm_sequence_" << sequence << "_n_split_" << n_split << "_corrected.csv";
  ofstream score_file(score_string.str());
  score_file << match_scores.format(CSVFormat);

  stringstream id_string;
  id_string << "ids_sequence_" << sequence << "_n_split_" << n_split << "_corrected.csv";
  ofstream id_file(id_string.str());
  id_file << match_ids.format(CSVFormat);

  // we can save the database. The created file includes the vocabulary
  // and the entries added
  // cout << "Saving database..." << endl;
  // stringstream ss;
  // ss << "orb_db_seq_" << sequence << "_n_split_" << n_split << ".yml.gz";
  // db.save(ss.str());
  cout << "... done!" << endl;

}

// ----------------------------------------------------------------------------


