#include <iostream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <System.h>

using namespace std;
using namespace cv;
using namespace openni;

enum
{
    // Camera
    LIPS_STREAM_PROPERTY_FOCAL_LENGTH_X     = 200,
    LIPS_STREAM_PROPERTY_FOCAL_LENGTH_Y     = 201,
    LIPS_STREAM_PROPERTY_PRINCIPAL_POINT_X  = 202,
    LIPS_STREAM_PROPERTY_PRINCIPAL_POINT_Y  = 203
};

/*
void showdevice()
{
    Array<DeviceInfo> aDeviceList;
    OpenNI::enumerateDevices(&aDeviceList);
    
    cout << "Computer connect to " << aDeviceList.getSize() << " sensors." << endl;
    
    for (int i=0; i<aDeviceList.getSize(); i++)
    {
	const DeviceInfo &rDevInfo = aDeviceList[i];
	cout << "Device: " << rDevInfo.getName() << endl;
    }
}
*/

Status initstream(Status& rc, Device& LIPS, VideoStream& vsDepth, VideoStream& vsColor)
{
    // Depth stream.
    VideoMode mModeDepth;
    rc = vsDepth.create(LIPS, SENSOR_DEPTH);
    if (STATUS_OK != rc)
    {
	cout << "ERROR: Cannot create depth stream on device." << endl << endl;
    }
    else
    {
	mModeDepth = vsDepth.getVideoMode();
	cout << "Depth VideoMode: " << mModeDepth.getResolutionX() << " x " << mModeDepth.getResolutionY() << " @ " << mModeDepth.getFps() << " FPS";
	cout << ", Unit is ";
	if (mModeDepth.getPixelFormat() == PIXEL_FORMAT_DEPTH_1_MM)
	    cout << "1mm.";
	else if (mModeDepth.getPixelFormat() == PIXEL_FORMAT_DEPTH_100_UM)
	    cout << "100um.";
	cout << endl;
	
	vsDepth.setMirroringEnabled(false);
	rc = vsDepth.start();
	if (STATUS_OK != rc)
	{
	    cout << "ERROR: Cannot start depth stream on device." << endl << endl;
	}
    }
    
    // Color stream.
    VideoMode mModeColor;
    rc = vsColor.create(LIPS, SENSOR_COLOR);
    if (STATUS_OK != rc)
    {
	cout << "ERROR: Cannot create color stream on device." << endl << endl;
    }
    else
    {
	mModeColor = vsColor.getVideoMode();
	cout << "color VideoMode: " << mModeColor.getResolutionX() << " x " << mModeColor.getResolutionY() << " @ " << mModeColor.getFps() << " FPS";
	cout << ", Unit is ";
	if (mModeColor.getPixelFormat() == PIXEL_FORMAT_DEPTH_1_MM)
	    cout << "1mm.";
	else if (mModeColor.getPixelFormat() == PIXEL_FORMAT_DEPTH_100_UM)
	    cout << "100um.";
	cout << endl;
	
	vsColor.setMirroringEnabled(false);
	rc = vsColor.start();
	if (STATUS_OK != rc)
	{
	    cout << "ERROR: Cannot start color stream on device." << endl << endl;
	}
    }
    
    float hfov, vfov;
    double fx, fy, cx, cy;
    
    rc = vsDepth.getProperty<float>(ONI_STREAM_PROPERTY_HORIZONTAL_FOV, &hfov);
    rc = vsDepth.getProperty<float>(ONI_STREAM_PROPERTY_VERTICAL_FOV, &vfov);

    rc = vsDepth.getProperty(LIPS_STREAM_PROPERTY_FOCAL_LENGTH_X, &fx);
    rc = vsDepth.getProperty(LIPS_STREAM_PROPERTY_FOCAL_LENGTH_Y, &fy);
    rc = vsDepth.getProperty(LIPS_STREAM_PROPERTY_PRINCIPAL_POINT_X, &cx);
    rc = vsDepth.getProperty(LIPS_STREAM_PROPERTY_PRINCIPAL_POINT_Y, &cy);
    
    cout << "Calibration parameter: fx = " << fx << ", fy = " << fy << ", cx = " << cx << ", cy = " << cy << endl;
    
    if (!vsColor.isValid() || !vsDepth.isValid())
    {
        cerr << "Depth or Color stream is invalid!" << endl;
        OpenNI::shutdown();
        rc = STATUS_ERROR;
        return rc;
    }

    return rc;
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cerr << endl << "Usage: ./rgbd_LIPS path_to_vocabulary path_to_settings" << endl;
        return 1;
    }
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);
    
    cout << endl << "-------" << endl;
    cout << "Openning LIPS ..." << endl;
    
    Status rc = STATUS_OK;
    // 初始化OpenNI環境.
    OpenNI::initialize();
    //showdevice();
    if ( STATUS_OK != OpenNI::initialize() )
    {
        cout << "After initialization: " << OpenNI::getExtendedError() << endl;
        return 1;
    }
    
    // 宣告并打開Device.
    Device LIPS;
    const char *deviceURL = openni::ANY_DEVICE;
    rc = LIPS.open(deviceURL);
    if (STATUS_OK != rc)
    {
        cout << "ERROR: Cannot open device: " << OpenNI::getExtendedError() << endl;
        return 1;
    }
    
    // Check and enable Depth-To-Color image registration.
    if(!LIPS.isImageRegistrationModeSupported( IMAGE_REGISTRATION_DEPTH_TO_COLOR ))
    {
        cout << "ERROR: ImageRegistration mode is not supported!" << endl << endl;
        return 1;
    }
    LIPS.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    
    VideoStream vsDepth;
    VideoStream vsColor;
    if (initstream(rc, LIPS, vsDepth, vsColor) == STATUS_OK)
    {
	cout << "Open LIPS Successfully!"<<endl;
    }
    else
    {
	cout << "Open LIPS Failed!"<<endl;
	return 0;
    }
    
    // Main Loop.
    cv::Mat imRGB, imD;
    bool ContinueorNot = true;
    // 循環讀取stream information並保存在VideoFrameRef.
    VideoFrameRef vfDepth;
    VideoFrameRef vfColor;
    for (double index = 1.0; ContinueorNot; index+=1.0)
    {
	rc = vsDepth.readFrame(&vfDepth);
	if (rc == STATUS_OK)
	{
	    imD = cv::Mat(vfDepth.getHeight(), vfDepth.getWidth(), CV_16UC1, (void*)vfDepth.getData());
	}
	
	rc = vsColor.readFrame(&vfColor);
	if (rc == STATUS_OK)
	{
	    imRGB = cv::Mat(vfColor.getHeight(), vfColor.getWidth(), CV_8UC3, (void*)vfColor.getData());
	}
	
	SLAM.TrackRGBD(imRGB, imD, index);
	
	char c  = cv::waitKey(5);
        switch(c)
        {
        case 'q':
	    break;
        case 'p':
            cv::waitKey(0);
            break;
        default:
            break;
        }
    }
    
    // Stop all threads.
    SLAM.Shutdown();
    
    // Save camera trajectory (Use Matlab to draw).
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    
    cv::destroyAllWindows();
    return 0;
}


