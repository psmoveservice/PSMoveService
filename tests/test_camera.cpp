#include "PSEyeVideoCapture.h"

using namespace cv;

int main(int, char**)
{
    PSEyeVideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    
    // Test settings some properties.
    //cap.set(CV_CAP_PROP_EXPOSURE, 150);
    std::cout << "get(CV_CAP_PROP_EXPOSURE): " << cap.get(CV_CAP_PROP_EXPOSURE) << std::endl;

    namedWindow("result",1);
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        if (!frame.empty())
        {
            imshow("result", frame);
            if(waitKey(30) >= 0) break;
        }
        
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}