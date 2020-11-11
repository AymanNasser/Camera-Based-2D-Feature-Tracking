# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.


## Statistical Analysis 

Using stats script.py file to automate all the possible combination of detector and descriptor pairs.
Using KNN approach with (k=2) and filtering ratio = 0.8

***Matched descriptors are between first 2 frames***

| Approach no. | Detector + Descriptor | Total Keypoints | Total Matches | Time(ms) |
|------------- | ------------- | ------------- | ------------- | ------------- |
| 1 | Shi-Tomasi + BRISK | 1370 | 84 | 172.205 ms |
| 2 | Shi-Tomasi + BRIEF | 1370 | 96 | 45.8341 ms |
| 3 | Shi-Tomasi + ORB | 1370 | 87 | 49.5279 ms |
| 4 | Shi-Tomasi + FREAK | 1370 | 68 | 116.639 ms |
| 5 | Shi-Tomasi + SIFT | 1370 | 112 | 89.8264 ms |
| 6 | Harris + BRISK | 248 | 11 | 131.395 ms |
| 7 | Harris + BRIEF | 248 | 12 | 65.9843 ms |
| 8 | Harris + ORB | 248 | 11 | 54.3331 ms |
| 9 | FAST + BRISK | 5063 | 213 | 153.694 ms |
| 10 | FAST + BRIEF | 5063 | 229 | 32.8021 ms |
| 11 | FAST + ORB | 5063 | 218 | 31.1273 ms |
| 12 | BRISK + BRISK | 2757 | 138 | 288.656 ms |
| 13 | BRISK + FREAK | 2757 | 112 | 248.02 ms |
| 14 | ORB + BRISK | 500 | 60 | 1250.05 ms |
| 15 | ORB + BRIEF | 500 | 40 | 126.089 ms |
| 16 | ORB + SIFT | 500 | 67 | 190.433 ms |
| 17 | AKAZE + BRIEF | 1351 | 108 | 160.151 ms |
| 18 | AKAZE + ORB | 1351 | 102 | 178.226 ms |
| 19 | AKAZE + SIFT | 1351 | 134 | 230.366 ms |
| 20 | SIFT + FREAK | 1438 | 58 | 489.327 ms |
| 21 | SIFT + SIFT | 1438 | 82 | 382.408 ms |


### Top three detectors 
| Approach no. | Detector + Descriptor | Total Keypoints | Total Matches | Time(ms) |
|------------- | ------------- | ------------- | ------------- | ------------- |
| 1 | FAST + BRIEF | 5063 | 229 | 32.8021 ms |
| 2 | FAST + ORB | 5063 | 218 | 31.1273 ms |
| 3 | Shi-Tomasi + BRIEF | 1370 | 96 | 45.8341 ms |










