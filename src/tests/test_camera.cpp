#include "PSEyeVideoCapture.h"

using namespace cv;

int main(int, char**)
{
    PSEyeVideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    
    namedWindow("result",1);
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
		int wk = -1;
		if (!frame.empty())
		{
			imshow("result", frame);
			wk = waitKey(30);
		}
        
		if (wk == 27)
		{
			break;
		}
		else if (wk >= 0)
		{
			// q=119, w=101, e=114, r=116, t=121, y=97, a=115, s=100, d=102, f=103, g=104, g=27

			int cap_prop = CV_CAP_PROP_EXPOSURE;
			double val_diff = 0;
			std::string prop_str("CV_CAP_PROP_EXPOSURE");
			
			// q/a for +/- exposure
			if ((wk == 119) || (wk == 115))
			{
				cap_prop = CV_CAP_PROP_EXPOSURE;
				prop_str = "CV_CAP_PROP_EXPOSURE";
				val_diff = (wk == 119) ? 0.1 : -0.1;
			}

			// w/s for +/- contrast
			if ((wk == 101) || (wk == 100))
			{
				cap_prop = CV_CAP_PROP_CONTRAST;
				prop_str = "CV_CAP_PROP_CONTRAST";
				val_diff = (wk == 101) ? 0.1 : -0.1;
			}

			// e/d for +/- contrast
			if ((wk == 114) || (wk == 102))
			{
				cap_prop = CV_CAP_PROP_GAIN;
				prop_str = "CV_CAP_PROP_GAIN";
				val_diff = (wk == 114) ? 0.1 : -0.1;
			}

			// r/f for +/- contrast
			if ((wk == 116) || (wk == 103))
			{
				cap_prop = CV_CAP_PROP_HUE;
				prop_str = "CV_CAP_PROP_HUE";
				val_diff = (wk == 116) ? 5 : 5;
			}

			double val = cap.get(cap_prop);
			std::cout << "Value of " << prop_str << " was " << val << std::endl;
			val += val_diff;
			cap.set(cap_prop, val);
			val = cap.get(cap_prop);
			std::cout << "Value of " << prop_str << " changed by " << val_diff << " to " << val << std::endl;
		}
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}