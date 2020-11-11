# SFND 2D Feature Tracking Writeup for Addressing all Ruberic Points

#### Data Buffer
Used deque for O(1) time complexity operations instead of erasing from a vector

<img src="images/deque.png" width="512" height="106" />

#### Key Points Detectors
Implemented HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT as shown below & 
the detector is called by `callDetector` function using a `string`

<img src="images/detectors.png" width="512" height="106" />

Key Points are removed using using rect.contains method to focus only
on the preceeding vehicle 

<img src="images/keypoints removal.png" width="512" height="106" />