#include <iostream>
#include <iomanip>
#include "PSMoveController.h"
#include "ServerLog.h"
#include "PSEyeVideoCapture.h"
#include "opencv2/opencv.hpp"
#include <algorithm>

// For sleep
#ifdef _WIN32
#include <cstdlib>
#else
#include <unistd.h>
#endif

const int HUE_RANGE = 20;
const int SAT_RANGE = 85;
const int VAL_RANGE = 85;

void bgr2hsv_min_max(cv::Scalar rgb_in, cv::Scalar& min_out, cv::Scalar& max_out)
{
    cv::Scalar hsv_in;
    cv::cvtColor(rgb_in, hsv_in, CV_BGR2HSV);
    
    min_out.val[0] = MIN(hsv_in.val[0] - HUE_RANGE, 0);  //0-180
    min_out.val[1] = MIN(hsv_in.val[1] - SAT_RANGE, 0);  //0-255
    min_out.val[2] = MIN(hsv_in.val[2] - VAL_RANGE, 0);  //0-255
    
    max_out.val[0] = MAX(hsv_in.val[0] + HUE_RANGE, 180);
    max_out.val[1] = MAX(hsv_in.val[0] + SAT_RANGE, 255);
    max_out.val[2] = MAX(hsv_in.val[0] + VAL_RANGE, 255);
}

int main()
{
    log_init("info");

    if (hid_init() == -1)
    {
        std::cerr << "Failed to initialize hidapi" << std::endl;
        return -1;
    }
    
    PSMoveController psmove;
    PSEyeVideoCapture cap(0); // open the default camera
    cv::namedWindow("result",1);
    

	std::cout << "Opening PSMoveController..." << std::endl;
	if (psmove.open() && cap.isOpened())
	{
        const PSMoveControllerState *psmstate= nullptr;

        psmove.poll();
        psmstate= static_cast<const PSMoveControllerState *>(psmove.getState());

		unsigned char r = 255;
		unsigned char g = 0;
		unsigned char b = 0;
        psmove.setLED(r, g, b);
        psmove.setRumbleIntensity(0);
        
        cv::Mat frame;
        cv::Mat gsFrame;
        cv::Scalar led_min, led_max;
        bgr2hsv_min_max(CvScalar(r, g, b), led_min, led_max);
        

		while (psmove.getIsBluetooth() && psmstate->Move != CommonControllerState::Button_DOWN)
		{
            psmove.poll();
            psmstate= static_cast<const PSMoveControllerState *>(psmove.getState());
            
            psmove.setLED(r, g, b); // TODO: Rate-limit LED update

            cap >> frame; // get a new frame from camera
            
            if (!frame.empty())
            {
                imshow("result", frame);
                cv::inRange(frame, led_min, led_max, gsFrame);
                
                //TODO: use convex hull if ellipse fit proves to be slow
                //http://docs.opencv.org/3.1.0/d7/d1d/tutorial_hull.html#gsc.tab=0
                
                std::vector<cv::Point> biggest_contour;
                std::vector<std::vector<cv::Point> > contours;
                cv::findContours(gsFrame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                
                for (auto it = contours.begin(); it != contours.end(); ++it) {
                    if (it == begin(contours) || it->size() > biggest_contour.size())
                    {
                        biggest_contour = *it;
                    }
                }
                
                if (biggest_contour.size() > 6)
                {
                    // TODO: Fit ellipse to contour
                    //
                }
            }

			int myw = 4;
			std::cout << '\r' <<
				"# " << std::setw(myw) << std::left << psmstate->RawSequence <<
				" A(1): " <<
				std::setw(myw) << std::right << psmstate->Accel[0][0] << "," <<
				std::setw(myw) << std::right << psmstate->Accel[0][1] << "," <<
				std::setw(myw) << std::right << psmstate->Accel[0][2] <<
				"; A(2): " <<
				std::setw(myw) << std::right << psmstate->Accel[1][0] << "," <<
				std::setw(myw) << std::right << psmstate->Accel[1][1] << "," <<
				std::setw(myw) << std::right << psmstate->Accel[1][2] <<
				"; G(1): " <<
				std::setw(myw) << std::right << psmstate->Gyro[0][0] << "," <<
				std::setw(myw) << std::right << psmstate->Gyro[0][1] << "," <<
				std::setw(myw) << std::right << psmstate->Gyro[0][2] <<
				"; G(2): " <<
				std::setw(myw) << std::right << psmstate->Gyro[1][0] << "," <<
				std::setw(myw) << std::right << psmstate->Gyro[1][1] << "," <<
				std::setw(myw) << std::right << psmstate->Gyro[1][2] <<
				"; M: " <<
				std::setw(myw) << std::right << psmstate->Mag[0] << "," <<
				std::setw(myw) << std::right << psmstate->Mag[1] << "," <<
				std::setw(myw) << std::right << psmstate->Mag[2] <<
				std::flush;

#ifdef _WIN32
			_sleep(5); // 5 msec
#else
            usleep(5000);
#endif
		}
        std::cout << std::endl;

        psmove.close();
	}
    
    // Tear-down hid api
    hid_exit();
    
    return 0;
}