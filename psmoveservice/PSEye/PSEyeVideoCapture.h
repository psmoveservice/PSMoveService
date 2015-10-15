#include "opencv2/opencv.hpp"

class PSEyeVideoCapture : public cv::VideoCapture {
public:
    PSEyeVideoCapture(int camindex)
    : cv::VideoCapture(camindex)
    {
        // do nothing.
    }
};