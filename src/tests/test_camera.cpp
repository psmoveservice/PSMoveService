#include "PSEyeVideoCapture.h"
#include "opencv2/opencv.hpp"
#include <algorithm>

const std::vector<int> known_keys = {113, 97, 119, 115, 101, 100, 114, 102, 116, 103};
// q, a, w, s, e, d, r, f, t, g

int main(int, char**)
{
    PSEyeVideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    
    cv::namedWindow("result",1);
    for(;;)
    {
        cv::Mat frame;
        cap >> frame; // get a new frame from camera
		int wk = -1;
		if (!frame.empty())
		{
			imshow("result", frame);
			wk = cv::waitKey(30);
		}
        
        if (wk == 27)  // Escape
		{
			break;
		}
		else if (std::find(known_keys.begin(), known_keys.end(), wk) != known_keys.end())
		{
			int cap_prop = CV_CAP_PROP_EXPOSURE;
			double val_diff = 0;
			std::string prop_str("CV_CAP_PROP_EXPOSURE");
			
			// q/a for +/- exposure
            // CL Eye [0 255]
			if ((wk == 113) || (wk == 97))
			{
				cap_prop = CV_CAP_PROP_EXPOSURE;
				prop_str = "CV_CAP_PROP_EXPOSURE";
				val_diff = (wk == 113) ? 10 : -10;
			}

			// w/s for +/- contrast
            // Note that, for CL EYE at least, changing contrast ALSO changes exposure
            // CL Eye [0 255]
			if ((wk == 119) || (wk == 115))
			{
				cap_prop = CV_CAP_PROP_CONTRAST;
				prop_str = "CV_CAP_PROP_CONTRAST";
				val_diff = (wk == 119) ? 1 : -1;
			}

			// e/d for +/- gain
            // For CL Eye, gain seems to be changing colour balance. 
			if ((wk == 101) || (wk == 100))
			{
				cap_prop = CV_CAP_PROP_GAIN;
				prop_str = "CV_CAP_PROP_GAIN";
				val_diff = (wk == 101) ? 4 : -4;
			}

			// r/f for +/- hue
			if ((wk == 114) || (wk == 102))
			{
				cap_prop = CV_CAP_PROP_HUE;
				prop_str = "CV_CAP_PROP_HUE";
				val_diff = (wk == 114) ? 1 : -1;
			}
            
            // t/g for +/- sharpness
            // For CL_Eye, 1 - sharpness
            if ((wk == 116) || (wk == 103))
            {
                cap_prop = CV_CAP_PROP_SHARPNESS;
                prop_str = "CV_CAP_PROP_SHARPNESS";
                val_diff = (wk == 116) ? 1 : -1;
            }

			double val = cap.get(cap_prop);
			std::cout << "Value of " << prop_str << " was " << val << std::endl;

			val += val_diff;
			cap.set(cap_prop, val);

			val = cap.get(cap_prop);
			std::cout << "Value of " << prop_str << " changed by " << val_diff << " and is now " << val << std::endl;
		}
        else if (wk > 0)
        {
            std::cout << "Unknown key has code " << wk << std::endl;
        }
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}