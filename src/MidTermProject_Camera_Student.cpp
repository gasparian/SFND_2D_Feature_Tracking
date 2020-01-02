/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

// https://docs.opencv.org/3.4/d5/d51/group__features2d__main.html
// https://docs.opencv.org/master/da/df5/tutorial_py_sift_intro.html

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    CircBuf<DataFrame> dataBuffer(dataBufferSize); // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.add(frame);

        cout << "\n#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image

        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        string detectorType = "AKAZE"; // HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if ( detectorType.compare("SHITOMASI") == 0 )
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if ( detectorType.compare("HARRIS") == 0 )
        {
            detKeypointsHarris(keypoints, imgGray, false);
        } 
        else 
        {
            detKeypointsModern(keypoints, imgGray, detectorType, false);
        }

        // only keep keypoints on the preceding vehicle (static bbox)
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            vector<cv::KeyPoint> kptTmp;
            for ( cv::KeyPoint& kpt : keypoints )
            {
                if ( (kpt.pt.x >= vehicleRect.x) && (kpt.pt.x <= (vehicleRect.x+vehicleRect.width)) &&
                     (kpt.pt.y >= vehicleRect.y) && (kpt.pt.y <= (vehicleRect.y+vehicleRect.height)) ) {
                        kptTmp.push_back(kpt);
                }
                
            }
            keypoints = kptTmp;
            cout << " NOTE: Keypoints have been limited by the ROI: " << keypoints.size() << endl;
        }

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << "; " << keypoints.size() << endl;
        }

        // calculate distribution of keypoints size
        bool getSizes = true;
        vector<float> neighborhoodSizes;
        float kptSizeMedian = -1;
        if ( getSizes ) {
            for ( cv::KeyPoint& kpt : keypoints ) {
                if (kpt.size)
                    neighborhoodSizes.push_back(kpt.size);
            }
            sort(neighborhoodSizes.begin(), neighborhoodSizes.end());
            kptSizeMedian = neighborhoodSizes[neighborhoodSizes.size() / 2];

            cout << "Detected kypoints median neighborhood size: " << kptSizeMedian << endl;
        }
        
        // push keypoints and descriptor for current frame to end of data buffer
        dataBuffer.getItem(1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// Descriptors: BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        string descriptorType = "SIFT"; // BRIEF, ORB, FREAK, AKAZE, SIFT
        descKeypoints(dataBuffer.getItem(1)->keypoints, dataBuffer.getItem(1)->cameraImg, descriptors, descriptorType);

        // push descriptors for current frame to end of data buffer
        dataBuffer.getItem(1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF"; // MAT_BF, MAT_FLANN
            string descriptorType = "DES_HOG"; // DES_BINARY (BRIEF, ORB, FREAK, AKAZE); DES_HOG (SIFT); Makes sense only in BF mode
            string selectorType = "SEL_KNN"; // SEL_NN, SEL_KNN

            matchDescriptors(dataBuffer.getItem(2)->keypoints, dataBuffer.getItem(1)->keypoints,
                             dataBuffer.getItem(2)->descriptors, dataBuffer.getItem(1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);

            // store matches in current data frame
            dataBuffer.getItem(1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = (dataBuffer.getItem(1)->cameraImg).clone();

                cv::drawMatches(dataBuffer.getItem(2)->cameraImg, dataBuffer.getItem(2)->keypoints,
                                dataBuffer.getItem(1)->cameraImg, dataBuffer.getItem(1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

    } // eof loop over all images

    return 0;
}
