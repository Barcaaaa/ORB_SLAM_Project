#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include <System.h>
#include <boost/format.hpp>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 6)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence start_index end_index" << endl;
        return 1;
    }
    
    cv::VideoCapture cap(argv[5]);
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);
    
    int start_index = atoi( argv[4] );
    int end_index = atoi( argv[5] );
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    
    // Main loop
    cv::Mat imRGB, imD;
    for (int index=start_index; index<end_index; index ++)
    {
	boost::format fmt ("%s/rgb_index/%d.png");
	imRGB = cv::imread( (fmt%argv[3]%index).str(), CV_LOAD_IMAGE_UNCHANGED);
        fmt = boost::format("%s/dep_index/%d.png");
	imD = cv::imread( (fmt%argv[3]%index).str(), CV_LOAD_IMAGE_UNCHANGED);
    }
    
    // Stop all threads
    cv::waitKey(0);

    SLAM.Shutdown();

    return 0;
}