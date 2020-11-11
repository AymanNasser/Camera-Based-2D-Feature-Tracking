/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <deque>
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

namespace matdes_params // Matching descriptors parameters 
{
    std::string MATCHER_BF = "MAT_BF";
    std::string MATCHER_FLANN = "MAT_FLANN";
    std::string DESC_BINARY = "DES_BIN";
    std::string DESC_NOT_BINARY = "DES_NBIN";
    std::string SELECT_NN = "SEL_NN";
    std::string SELECT_KNN = "SEL_KNN";
}


/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    std::string detectorType;
    std::string descriptorType; 
    
    // Init vars for stats script
    if(argc == 3)
    {
        detectorType = argv[1];
        descriptorType = argv[2]; 
    }
    else if (argc == 1)
    {
        detectorType = "AKAZE";
        descriptorType = "AKAZE"; 
    }
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    std::string dataPath = "../";

    // camera
    std::string imgBasePath = dataPath + "images/";
    std::string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    std::string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 1;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    std::deque<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = true;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */
    double t_stats = (double)cv::getTickCount();
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        std::ostringstream imgNumber;
        imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;
        std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;

        if(dataBuffer.size() > dataBufferSize)
            dataBuffer.pop_front();
        
        dataBuffer.push_back(frame);

        //// EOF STUDENT ASSIGNMENT
        //std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        bool bDetectorVis = false;

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
        callDetector(imgGray, detectorType, keypoints, bDetectorVis);
        
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        std::vector<cv::KeyPoint> filteredKeypoints; // filtered keypoints after removing non preceeding vehicle keypoints 
        cv::Rect vehicleRect(535, 180, 180, 150);
        //std::cout << "Keypoints size before bounding box removal: "<< keypoints.size() << std::endl;
        if (bFocusOnVehicle)
        {
            for(auto it = keypoints.begin(); it != keypoints.end(); it++){
                // Checking if the specified rectangle contains the keypoint coordinates &&
                // if the diameter of the keypoint not exceeding the width of vehicle
                if(vehicleRect.contains(it->pt) && (it->size) <= vehicleRect.width) 
                    filteredKeypoints.push_back(*it);
            } 
        }

        //std::cout << "Keypoints size after bounding box removal: "<< filteredKeypoints.size() << std::endl;
        
        /* cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, filteredKeypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "TEST";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0); */
       

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { 
                // there is no response info, so keep the first 50 as they are sorted in descending quality order
                filteredKeypoints.erase(filteredKeypoints.begin() + maxKeypoints, filteredKeypoints.end());
            }
            cv::KeyPointsFilter::retainBest(filteredKeypoints, maxKeypoints);
            //std::cout << "NOTE: Keypoints have been limited!" << std::endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = filteredKeypoints;
        
        //std::cout << "#2 : DETECT KEYPOINTS done" << std::endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        //std::cout << "#3 : EXTRACT DESCRIPTORS done" << std::endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            std::vector<cv::DMatch> matches;
            std::string matcherType = matdes_params::MATCHER_BF;         // MAT_BF, MAT_FLANN
            std::string descriptorType = matdes_params::DESC_BINARY;     // DES_BINARY, DES_HOG
            std::string selectorType = matdes_params::SELECT_KNN;         // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);
           
            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;
            std::cout << "No. of Key Point Matches= " << matches.size() << std::endl;
            //std::cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;

            // visualize matches between current and previous image
            bVis = false;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                std::string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                //std::cout << "Press key to continue to next image" << std::endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }
       // std::cout << "\n\n###################\n\n";

    } // eof loop over all images
    t_stats = ((double)cv::getTickCount() - t_stats) / cv::getTickFrequency();
    std::cout<< "Total Time=" << 1000 * t_stats / 1.0 << " ms" << std::endl;
    return 0;
}
