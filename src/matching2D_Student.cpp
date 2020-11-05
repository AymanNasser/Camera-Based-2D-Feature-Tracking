#include <numeric>
#include "matching2D.hpp"

// KeyPoints is a data structure for salient point detectors.
// The class instance stores a keypoint, i.e. a point feature found by one of many available keypoint detectors, such as Harris corner detector
// The keypoint is characterized by the 2D position, scale (proportional to the diameter of the neighborhood that needs to be taken into account), 
// orientation and some other parameters.


// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        // ...
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        // ...
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else
    {

        //...
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << std::endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / std::max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    
    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)


    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);

    auto startTime = std::chrono::steady_clock::now();
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled); // Scales, calculates absolute values, and converts the result to 8-bit
    auto endTime = std::chrono::steady_clock::now();

    float IoU_Threshold = 0;
    for(int rows=0; rows<dst_norm.rows; rows++){

        for(int cols=0; cols<dst_norm.cols; cols++){
            
            int pixelIntensity = dst_norm.at<float>(rows,cols);
            
            // If pixel intensity higher than the intensity threshold ==> take into account 
            if(pixelIntensity > minResponse)
            {
                cv::KeyPoint tempKeyPoint;
                tempKeyPoint.pt = cv::Point2f(cols,rows); // Columns --> change in x coord, Rows --> change in y coord  
                tempKeyPoint.response = pixelIntensity;
                tempKeyPoint.size = 2*apertureSize;

                // NMS procedure 
                bool isOverlapOccurred = false;
                for(auto it = keypoints.begin(); it != keypoints.end(); it++){
                    
                    /* This method computes overlap for pair of keypoints. Overlap is the ratio between area of keypoint regions' 
                    * intersection and area of keypoint regions' union (considering keypoint region as circle). If they don't overlap, we get zero. 
                    * If they coincide at same location with same size, we get 1
                    */
                    float overlapArea = cv::KeyPoint::overlap(tempKeyPoint, *it);

                    if(overlapArea > IoU_Threshold)
                    {   
                        isOverlapOccurred = true;
                        if(tempKeyPoint.response > it->response)
                            *it = tempKeyPoint; // Replacing the old keypoint with the new keypoint due to higher intensity of the new keypoint 
                    }
                }

                if(! isOverlapOccurred )
                    keypoints.push_back(tempKeyPoint);
            }

        }

    }

    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Corner Harris KeyPoint Detection Took: " << elapsedTime.count() << " milliseconds" << std::endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsSIFT(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{

}
void detKeypointsFAST(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{

}
void detKeypointsBRISK(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{

}



