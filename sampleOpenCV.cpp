/****************************************************************************\
 * Copyright (C) 2022 Infineon Technologies & pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#include <royale.hpp>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>

#include <sample_utils/PlatformResources.hpp>

using namespace royale;
using namespace sample_utils;
using namespace std;
using namespace cv;

// define images that we want to display globally, so that we can access them from all threads
Mat scaledZImage, imgCannyDepth, imgCanny, imgBlurred;

condition_variable newData;
mutex mut;
boolean ready = false;

class MyListener : public IDepthDataListener
{

public :

    MyListener() :
        undistortImage (false)
    {
    }

    void onNewData (const DepthData *data)
    {
        // this callback function will be called for every new
        // depth frame

        std::lock_guard<std::mutex> lock (flagMutex);

        unique_lock<mutex> ul (mut);

        // define images for depth and gray and for their 8Bit and scaled versions, the images we show are defined globally
        Mat zImage, zImage8;
        Mat grayImage, grayImage8, scaledGrayImage;
        Mat imgBlurredDepth;

        // create two images which will be filled afterwards
        // each image containing one 32Bit channel
        zImage.create (Size (data->width, data->height), CV_32FC1);
        grayImage.create (Size (data->width, data->height), CV_32FC1);

        // set the image to zero
        zImage = Scalar::all (0);
        grayImage = Scalar::all (0);

        int k = 0;
        for (int y = 0; y < zImage.rows; y++)
        {
            float *zRowPtr = zImage.ptr<float> (y);
            float *grayRowPtr = grayImage.ptr<float> (y);
            for (int x = 0; x < zImage.cols; x++, k++)
            {
                auto curPoint = data->points.at (k);
                if (curPoint.depthConfidence > 0)
                {
                    // if the point is valid, map the pixel from 3D world
                    // coordinates to a 2D plane (this will distort the image)
                    zRowPtr[x] = adjustZValue (curPoint.z);
                    grayRowPtr[x] = static_cast<float> (curPoint.grayValue);
                }
            }
        }

        // create images to store the 8Bit version (some OpenCV
        // functions may only work on 8Bit images)
        zImage8.create (Size (data->width, data->height), CV_8UC1);
        grayImage8.create (Size (data->width, data->height), CV_8UC1);

        // convert images to the 8Bit version
        // This sample uses a fixed scaling of the values to (0, 255) to avoid flickering.
        normalize (grayImage, grayImage8, 50, 255, NORM_MINMAX, CV_8UC1);
        zImage.convertTo (zImage8, CV_8UC1);

        if (undistortImage)
        {
            // call the undistortion function on the z image
            Mat temp = zImage8.clone();
            undistort (temp, zImage8, cameraMatrix, distortionCoefficients);
        }

        // scale the depth image
        scaledZImage.create (Size (data->width * 4, data->height * 4), CV_8UC1);
        resize (zImage8, scaledZImage, scaledZImage.size());

        if (undistortImage)
        {
            // call the undistortion function on the gray image
            Mat temp = grayImage8.clone();
            undistort (temp, grayImage8, cameraMatrix, distortionCoefficients);
        }

        // scale the gray image
        scaledGrayImage.create (Size (data->width * 4, data->height * 4), CV_8UC1);
        resize (grayImage8, scaledGrayImage, scaledGrayImage.size());

        cv::GaussianBlur (scaledGrayImage, // input image
                          imgBlurred,      // output image
                          cv::Size (11, 11), // smoothing window width and height in pixels
                          150.5);            // sigma value, determines how much the image will be blurred

        cv::Canny (imgBlurred, // input image
                   imgCanny,   // output image
                   15, 25);

        cv::GaussianBlur (scaledZImage,    // input image
                          imgBlurredDepth, // output image
                          cv::Size (5, 5), // smoothing window width and height in pixels
                          5.5);            // sigma value, determines how much the image will be blurred

        cv::Canny (imgBlurredDepth, // input image
                   imgCannyDepth,   // output image
                   10, 20);

        ready = true;
        ul.unlock();
        newData.notify_one();
        ul.lock();
        newData.wait (ul, []()
        {
            return ready == false;
        });
    }

    void setLensParameters (const LensParameters &lensParameters)
    {
        // Construct the camera matrix
        // (fx   0    cx)
        // (0    fy   cy)
        // (0    0    1 )
        cameraMatrix = (Mat1d (3, 3) << lensParameters.focalLength.first, 0, lensParameters.principalPoint.first,
                        0, lensParameters.focalLength.second, lensParameters.principalPoint.second,
                        0, 0, 1);

        // Construct the distortion coefficients
        // k1 k2 p1 p2 k3
        distortionCoefficients = (Mat1d (1, 5) << lensParameters.distortionRadial[0],
                                  lensParameters.distortionRadial[1],
                                  lensParameters.distortionTangential.first,
                                  lensParameters.distortionTangential.second,
                                  lensParameters.distortionRadial[2]);
    }

    void toggleUndistort()
    {
        std::lock_guard<std::mutex> lock (flagMutex);
        undistortImage = !undistortImage;
    }

private:

    // adjust z value to fit fixed scaling, here max dist is 2.5m
    // the max dist here is used as an example and can be modified
    float adjustZValue (float zValue)
    {
        float clampedDist = std::min (2.5f, zValue);
        float newZValue = clampedDist / 2.5f * 255.0f;
        return newZValue;
    }

    // lens matrices used for the undistortion of the image
    Mat cameraMatrix;
    Mat distortionCoefficients;

    std::mutex flagMutex;
    bool undistortImage;
};

int main (int argc, char *argv[])
{
    // Windows requires that the application allocate these, not the DLL.
    PlatformResources resources;

    // This is the data listener which will receive callbacks.  It's declared
    // before the cameraDevice so that, if this function exits with a 'return'
    // statement while the camera is still capturing, it will still be in scope
    // until the cameraDevice's destructor implicitly de-registers the listener.
    MyListener listener;

    // this represents the main camera device object
    std::unique_ptr<ICameraDevice> cameraDevice;

    // the camera manager will query for a connected camera
    {
        CameraManager manager;

        // check the number of arguments
        if (argc > 1)
        {
            // if the program was called with an argument try to open this as a file
            cout << "Trying to open : " << argv[1] << endl;
            cameraDevice = manager.createCamera (argv[1]);
        }
        else
        {
            // if no argument was given try to open the first connected camera
            royale::Vector<royale::String> camlist (manager.getConnectedCameraList());
            cout << "Detected " << camlist.size() << " camera(s)." << endl;

            if (!camlist.empty())
            {
                cameraDevice = manager.createCamera (camlist[0]);
            }
            else
            {
                cerr << "No suitable camera device detected." << endl
                     << "Please make sure that a supported camera is plugged in, all drivers are "
                     << "installed, and you have proper USB permission" << endl;
                return 1;
            }

            camlist.clear();
        }
    }
    // the camera device is now available and CameraManager can be deallocated here

    if (cameraDevice == nullptr)
    {
        // no cameraDevice available
        if (argc > 1)
        {
            cerr << "Could not open " << argv[1] << endl;
            return 1;
        }
        else
        {
            cerr << "Cannot create the camera device" << endl;
            return 1;
        }
    }

    // IMPORTANT: call the initialize method before working with the camera device
    auto status = cameraDevice->initialize();
    if (status != CameraStatus::SUCCESS)
    {
        cerr << "Cannot initialize the camera device, error string : " << getErrorString (status) << endl;
        return 1;
    }

    // retrieve the lens parameters from Royale
    LensParameters lensParameters;
    status = cameraDevice->getLensParameters (lensParameters);
    if (status != CameraStatus::SUCCESS)
    {
        cerr << "Can't read out the lens parameters" << endl;
        return 1;
    }

    listener.setLensParameters (lensParameters);

    // register a data listener
    if (cameraDevice->registerDataListener (&listener) != CameraStatus::SUCCESS)
    {
        cerr << "Error registering data listener" << endl;
        return 1;
    }

    // start capture mode
    if (cameraDevice->startCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error starting the capturing" << endl;
        return 1;
    }

    int currentKey = 0;

    while (currentKey != 27)
    {
        // wait until a key is pressed
        currentKey = waitKey (10) & 255;

        if (currentKey == 'd')
        {
            // toggle the undistortion of the image
            listener.toggleUndistort();
        }

        unique_lock<mutex> ul (mut);
        newData.wait (ul, []()
        {
            return ready;
        });

        imshow ("Depth", scaledZImage);
        imshow ("Gray", imgBlurred);
        imshow ("Canny", imgCanny);
        imshow ("Canny Depth", imgCannyDepth);

        ready = false;
        ul.unlock();
        newData.notify_one();
    }

    // stop capture mode
    if (cameraDevice->stopCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error stopping the capturing" << endl;
        return 1;
    }

    return 0;
}

