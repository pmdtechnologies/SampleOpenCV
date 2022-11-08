# Tutorial - OpenCV Example
This tutorial explains the "sampleOpenCV" which shows how you can use [OpenCV](https://docs.opencv.org/4.6.0/) with Royale. The processing of the images is done in a different thread.

## Installation
To run this sample you need to install a Royale binary version and additionaly the OpenCV library. This sample was tested with OpenCV version 4.6.0.

After installing everything, you can start CMake. There you have to set the OpenCV_DIR to your OpenCV installation (e.g. C:/projects/opencv-4.6.0/opencv/build) 
and the royale_DIR to the share folder of your Royale binary installation (e.g. D:\Program Files\royale\4.23.0.1062\share) and click **Generate**.

## Code explanation
In the beginning we declare a data listener and a camera device. We also declare the platform resources as this will call the CoInitializeEx function on Windows. Otherwise 
we won't be able to use camera devices that use the UVC standard.
```cpp
PlatformResources resoures;
MyListener listener;
std::unique_ptr<ICameraDevice> cameraDevice;
```

### 1 Camera Setup
First we use a CameraManager to **create the camera device:** if you passed a path to an .rrf-file via the commandline this file will be used, else the camera
manager will use a connected camera. If more than one camera is connected, the first one is used. If no camera is detected or a problem occurs, an error
will be thrown. After this we **initialize the camera device**.

```cpp
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
```

### 2 Data Listener Setup
To setup our custom listener, we first **set its lens parameters**. The listener is of the type MyListener which is a IDepthDataListener, that means it 
receives the depth data of a camera device. It has a public function [setLensParameters](######setLensParameters) which we use to set the parameters 
we retrieved from the camera.
Next we **register the listener** with Royale. 

```cpp
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
```

###### setLensParameters
The public method setLensParameters of MyListener gets the intrinsic camera lens parameters and constructs the camera matrix and the distortion coeficients from them. This will be needed
if we want to undistort the image with OpenCV.

```cpp
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
```

### 3 Start capturing
The next step is to start the capturing of the camera device.

```cpp 
    // start capture mode
    if (cameraDevice->startCapture() != CameraStatus::SUCCESS)
    {
        cerr << "Error starting the capturing" << endl;
        return 1;
    }

``` 

### 4 Show OpenCV-processed images
We show the processed images in a while loop. We break out of the loop when the escape-key (keycode 27) gets pressed. If the key "d" is pressed, the undistortion 
of the image is toggled (see [toggleUndistort](######toggleUndistort) and [cv::undistort documentation](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html). 
When the listeners [onNewData](######onNewData) notifies us that new images are available, we display them. 
Shown are four images: the depth image, the blurred gray image, the gray image with canny edge detection and the depth image with canny edge detection. 

When we break out of the while-loop, we stop the capturing and the program finishes.

**Note** that with OpenCV all UI functions, like `imshow()` must be called from the main thread. Because of this we implemented a condition_variable 
`newData` which in combination with a mutex `mut` and the boolean `ready` determines if the images can be displayed in the `main` or not. We also have to
define these and the images globally, so that we can access them from all threads. 

```cpp
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

        unique_lock<mutex> ul(mut);
        newData.wait(ul, []() {return ready; });

        imshow("Depth", scaledZImage);
        imshow("Gray", imgBlurred);
        imshow("Canny", imgCanny);
        imshow("Canny Depth", imgCannyDepth);
        
        ready = false;
        ul.unlock();
        newData.notify_one();
    }
      
```

**On the data listener side this is happening:**
The processing of the images with OpenCV happens in [onNewData](######onNewData).

###### onNewData
This public method gets a pointer to the captured Depthdata and will be called for every new frame. Inside this method the images are created and filled
with data. After that they are processed. The processing consists of applying [gaussian blur](https://docs.opencv.org/4.x/d4/d86/group__imgproc__filter.html#gaabe8c836e97159a9193fb0b11ac52cf1) 
and [canny edge detection](https://docs.opencv.org/4.x/da/d22/tutorial_py_canny.html) to the gray value image and the depth
image. But before we can do that we need to convert the images to 8 Bit and scale them. If the undistortion is activated, it will happen here.

We use the same mutex, condition_variable and mutex as in `main` to handle if the images are ready to be displayed in the other thread. `ready`is set to true, the 
mutex is unlocked and main gets notified by the condition_variable that new processed images are available.

```cpp
    void onNewData (const DepthData *data)
    {
        // this callback function will be called for every new
        // depth frame

        std::lock_guard<std::mutex> lock (flagMutex);

        unique_lock<mutex> ul(mut);

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
        newData.wait(ul, []() {return ready == false; });
    }
```

###### toggleUndistort
The public function toggleUndistort sets the value of the boolean `undistortImage` to its opposite. If it was `true` it will be `false` and vice versa.

```cpp
    void toggleUndistort()
    {
        std::lock_guard<std::mutex> lock (flagMutex);
        undistortImage = !undistortImage;
    }
```

###### from depth data to image
To be able to show the 3D depth data as 2D images, we need to **map to a 2D plane**. This happens in the two lines below (or inside of [onNewData](######onNewData). 
The function adjustZValue is part of the MyListener class. Here we assume that every z-value lies between 0 and 2.5 meters (if you need more depth you can adjust 
this value in the code), if a z-value is bigger than 2.5 meters it will be set to the maximum. 
The x and y coordinates of the point are discarded. So in the end we fill the images with the adjusted z-value (zImage) and the gray value (grayImage) we derive from it.

```cpp
    // if the point is valid, map the pixel from 3D world coordinates to a 2D plane (this will distort the image)
    zRowPtr[x] = adjustZValue (curPoint.z);
    grayRowPtr[x] = static_cast<float> (curPoint.grayValue);
```
