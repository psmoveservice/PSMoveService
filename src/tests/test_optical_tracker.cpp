#include <iostream>
#include <iomanip>
#include "PSMoveController.h"
#include "ServerLog.h"
#include "PSEyeVideoCapture.h"
#include "opencv2/opencv.hpp"
#include <algorithm>
#include <Eigen/Dense>

#ifndef HAVE_M_PI
#ifndef M_PI
#define M_PI    3.14159265358979323846264338327950288   /* pi */
#endif
#endif

const int HUE_RANGE = 25;
const int SAT_RANGE = 50;
const int VAL_RANGE = 32;

typedef cv::Vec< unsigned char, 3 > cvBGR;
typedef cv::Vec< unsigned char, 3 > cvRGB;
typedef cv::Vec< unsigned char, 3 > cvHSV;

cvHSV
bgr2hsv(cvBGR bgr_in)
{
    static cv::Mat img_bgr(1, 1, CV_8UC3, bgr_in);
    static cv::Mat img_hsv(1, 1, CV_8UC3);
    cv::cvtColor(img_bgr, img_hsv, CV_BGR2HSV);
    return img_hsv.at<cvHSV>(0, 0);
}

void hsv_range(cvHSV hsv_in, cvHSV& min_out, cvHSV& max_out)
{
    min_out.val[0] = MAX(hsv_in.val[0] - HUE_RANGE, 0);  //0-180
    min_out.val[1] = MAX(hsv_in.val[1] - SAT_RANGE, 0);  //0-255
    min_out.val[2] = MAX(hsv_in.val[2] - VAL_RANGE, 0);  //0-255
    
    max_out.val[0] = MIN(hsv_in.val[0] + HUE_RANGE, 180);
    max_out.val[1] = MIN(hsv_in.val[1] + SAT_RANGE, 255);
    max_out.val[2] = MIN(hsv_in.val[2] + VAL_RANGE, 255);
}

void onMouse(int evt, int x, int y, int flags, void* param) {
    if(evt == CV_EVENT_LBUTTONDOWN) {
        cv::Point *ptPtr = (cv::Point*)param;
        *ptPtr = cv::Point(x, y);
    }
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
    cv::namedWindow("bgr video");
    cv::namedWindow("hsv video");
    cv::namedWindow("result");
    
    cv::Point clickPoint;
    cv::setMouseCallback("hsv video", onMouse, (void*)&clickPoint);

	std::cout << "Opening PSMoveController..." << std::endl;
	if (psmove.open() && cap.isOpened())
	{
        const PSMoveControllerState *psmstate= nullptr;

        cap.set(cv::CAP_PROP_EXPOSURE, 16);
        cap.set(cv::CAP_PROP_GAIN, 64);

        psmove.poll();
        psmstate= static_cast<const PSMoveControllerState *>(psmove.getState());

		unsigned char r = 255;
		unsigned char g = 0;
		unsigned char b = 255;
        psmove.setLED(r, g, b);
        psmove.setRumbleIntensity(0);
        
        cv::Mat bgrFrame;
        cv::Mat hsvFrame;
        cv::Mat intGsFrame;
        cv::Mat gsFrame;
        cvHSV led_min, led_max;
        cvHSV psmoveHSVColour = bgr2hsv(cvBGR(b, g, r));
        
        // HACK FOR TESTING
        psmoveHSVColour = cvHSV(140, 25, 255);
        
        hsv_range(psmoveHSVColour, led_min, led_max);
        

		while (psmove.getIsBluetooth() && psmstate->Move != CommonControllerState::Button_DOWN)
		{
            psmove.poll();
            psmstate= static_cast<const PSMoveControllerState *>(psmove.getState());
            
            psmove.setLED(r, g, b); // TODO: Rate-limit LED update

            cap >> bgrFrame; // get a new frame from camera
            
            if (!bgrFrame.empty())
            {
                cv::cvtColor(bgrFrame, hsvFrame, CV_BGR2HSV);       // Convert frame to HSV colour space
                cv::inRange(hsvFrame, led_min, led_max, gsFrame);  // Filter based on led HSV range
                
//                cv::GaussianBlur(intGsFrame, gsFrame, cv::Size(3, 3), 0);
//                cv::blur(intGsFrame, gsFrame, cv::Size(3, 3));  // Fastest
//                cv::bilateralFilter(intGsFrame, gsFrame, 3, 6.0, 1.5);  // Preserves edges
                
                imshow("result", gsFrame);
                
//                if (clickPoint.x > 0)
//                {
//                    cvHSV hsv_bulb = hsvFrame.at<cvHSV>(clickPoint);
//                    std::cout << clickPoint << ";" << hsv_bulb << std::endl;
//                }
                
                std::vector<std::vector<cv::Point> > contours;
                cv::findContours(gsFrame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
                
                cv::Scalar contCol = cv::Scalar( 255, 0, 0 );
                for (int i=0; i<contours.size(); ++i) {
                    cv::drawContours(bgrFrame, contours, i, contCol);
                }

                std::vector<cv::Point> biggest_contour;
                if (contours.size() > 0)
                {
                    double contArea = 0;
                    double newArea = 0;
                    for (auto it = contours.begin(); it != contours.end(); ++it) {
                        newArea = cv::contourArea(*it);
                        if (newArea > contArea)
                        {
                            contArea = newArea;
                            biggest_contour = *it;
                        }
                    }
                }
                
                //TODO: If our contour is suddenly much smaller than last frame,
                // but is next to an almost-as-big contour, then maybe these
                // 2 contours should be joined.
                // (i.e. if a finger is blocking the middle of the bulb)

                if (biggest_contour.size() > 6)
                {
                    // Remove any points in contour on edge of camera/ROI
                    std::vector<cv::Point>::iterator it = biggest_contour.begin();
                    while (it != biggest_contour.end()) {
                        if (it->x == 320 || it->x == -320 || it->y == 240 || it->y == -240) {
                            it = biggest_contour.erase(it);
                        }
                        else {
                            ++it;
                        }
                    }

                    // Get the convex hull
                    std::vector<cv::Point> hull;
                    cv::convexHull(biggest_contour, hull);
                    

//                    // If our hull has lots of points, select the N consecutive
//                    // points that are furthest, because furthest are least
//                    // likely to be occluded (assuming player is near middle of FOV).
//                    int N = 10;
//                    if (hull.size() > N)
//                    {
//                        std::deque<float> dists;
//                        float max_dist = 0;
//                        float sum_dists = 0;
//                        int best_i = 0;
//                        for (int i=0; i < (hull.size()-N); i++) {
//                            float new_dist = hull[i].x*hull[i].x + hull[i].y*hull[i].y;
//                            sum_dists += new_dist;
//                            dists.push_back(new_dist);
//                            if (i >= N)
//                            {
//                                sum_dists -= dists.front();
//                                dists.pop_front();
//                                if (sum_dists > max_dist)
//                                {
//                                    best_i = i;
//                                    max_dist = sum_dists;
//                                }
//                            }
//                        }
//                        hull = std::vector<cv::Point>(&hull[best_i], &hull[best_i+N]);
//                    }
                    
                    {
                        std::vector< std::vector<cv::Point> > contours;
                        
                        contours.push_back(hull);
                        cv::drawContours(bgrFrame, contours, 0, cv::Scalar(255, 0, 255));
                    }

                    
                    // Fit ellipse to reduced hull
                    
                    // Subtract midpoint from each point.
                    std::for_each(hull.begin(), hull.end(), [](cv::Point& p) { p.x -= 320; p.y = 240 - p.y;});
                    
                    
                    
                    if (hull.size() > 5)
                    {
                        // http://autotrace.sourceforge.net/WSCG98.pdf
                        std::vector<float> conic_params(6);
                        
                        Eigen::MatrixXf D1(hull.size(), 3);
                        Eigen::MatrixXf D2(hull.size(), 3);
                        for (int ix=0; ix < hull.size(); ++ix) {
                            D1.row(ix)[0] = hull[ix].x * hull[ix].x;
                            D1.row(ix)[1] = hull[ix].x * hull[ix].y;
                            D1.row(ix)[2] = hull[ix].y * hull[ix].y;
                            D2.row(ix)[0] = hull[ix].x;
                            D2.row(ix)[1] = hull[ix].y;
                            D2.row(ix)[2] = 1;
                        }
                        // std::cout << "\n\n\n\n" << hull << "\n\n\n\n" << std::endl;
                        Eigen::Matrix3f S1 = D1.transpose() * D1;
                        Eigen::Matrix3f S2 = D1.transpose() * D2;
                        Eigen::Matrix3f S3 = D2.transpose() * D2;
                        Eigen::Matrix3f T = -S3.colPivHouseholderQr().solve(S2.transpose());
//                        Eigen::Matrix3f T = -S3.inverse() * S2.transpose();
                        Eigen::Matrix3f M = S2*T + S1;
                        Eigen::Matrix3f Mout;
                        Mout.block<1, 3>(0, 0) = M.block<1, 3>(2, 0) / 2;
                        Mout.block<1, 3>(1, 0) = -M.block<1, 3>(1, 0);
                        Mout.block<1, 3>(2, 0) = M.block<1, 3>(0, 0) / 2;
                        Eigen::EigenSolver<Eigen::Matrix3f> eigsolv(Mout);
                        for (int row_ix=0; row_ix<3; ++row_ix) {
                            Eigen::Vector3f evec = eigsolv.eigenvectors().col(row_ix).real();
                            //cond = 4 * evec(1, :) .* evec(3, :) - evec(2, :).^2; % evaluate a'Ca
                            if ((4.0 * evec[0] * evec[2]) - (evec[1] * evec[1]) > 0)
                            {
                                conic_params[0] = evec[0];
                                conic_params[1] = evec[1];
                                conic_params[2] = evec[2];
                                Eigen::Vector3f Tevec = T*evec;
                                conic_params[3] = Tevec[0];
                                conic_params[4] = Tevec[1];
                                conic_params[5] = Tevec[2];
                            }
                        }

                        cv::RotatedRect cvFitEllipse= cv::fitEllipse(hull);
                        cv::ellipse(
                            bgrFrame, 
                            cv::Point(cvFitEllipse.center.x + 320, 240 - cvFitEllipse.center.y),
                            cv::Size(cvFitEllipse.size.width/2, cvFitEllipse.size.height/2),
                            cvFitEllipse.angle,
                            0, 360, 
                            cv::Scalar(0, 255, 255));
                        
                        // Convert to parametric params
                        float A = conic_params[0];
                        float B = conic_params[1] / 2;
                        float C = conic_params[2];
                        float D = conic_params[3] / 2;
                        float F = conic_params[4] / 2;
                        float G = conic_params[5];
                        float BB = B*B;
                        float AC = A*C;
                        float FF = F*F;
                        float off_d = BB - AC;
                        float h = (C*D - B*F) / off_d;
                        float k = (A*F - B*D) / off_d;
                        float semi_n = A*FF + C*D*D + G*BB - 2*B*D*F - AC*G;
                        float dAC = A-C;
                        float dACsq = dAC*dAC;
                        float semi_d_2 = sqrtf(dACsq + 4*BB);
                        float semi_d_3 = -A - C;
                        float a_sqrd = (2 * semi_n) / (off_d * (semi_d_3 + semi_d_2));
                        float b_sqrd = (2 * semi_n) / (off_d * (semi_d_3 - semi_d_2));

                        if (a_sqrd > FLT_EPSILON && b_sqrd > FLT_EPSILON)
                        {
                            // b and tau are only needed for drawing an ellipse.
                            float a = sqrt(a_sqrd);
                            float b = sqrt(b_sqrd);
                            double tau = atan2(2 * B, dAC) / 2;  //acot((A-C)/(2*B))/2;
                            if (A > C)
                            {
                                tau += M_PI/2;
                            }
//                            if (tau < 0)
//                            {
//                                float oldb = b;
//                                b = a;
//                                a = oldb;
//                                tau += M_PI;
//                            }
                            cv::ellipse(bgrFrame, cv::Point(h+320, 240-k), cv::Size(a, b), tau, 0, 360, cv::Scalar(0, 255, 0));

                            //Get sphere coordinates from parametric
                            float F_PX = 554.2563;
                            float R = 2.25;
                            float L_px = sqrt(h*h + k*k);
                            float m = L_px / F_PX;
                            float fl = F_PX / L_px;
                            float j = (L_px + a) / F_PX;
                            float l = (j - m) / (1 + j*m);
                            float D_cm = R * sqrt(1 + l*l) / l;
                            float z = D_cm * fl / sqrt(1 + fl*fl);
                            float L_cm = z * m;
                            float x = L_cm * h / L_px;
                            float y = L_cm * k / L_px;

                            std::cout << "X: " << x << " Y: " << y << " Z: " << z << std::endl;
                        }
                    }
                    
                }

                imshow("bgr video", bgrFrame);
                imshow("hsv video", hsvFrame);
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

            cv::waitKey(16);
		}
        std::cout << std::endl;

        psmove.close();
	}
    
    // Tear-down hid api
    hid_exit();
    
    return 0;
}