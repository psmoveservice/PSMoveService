#include "PSEyeVideoCapture.h"
#include "ClientConstants.h"
#include "opencv2/opencv.hpp"
#include <algorithm>
#include <vector>
#include <chrono>
#include <ppl.h>
#include <concurrent_vector.h>

const std::vector<int> known_keys = { 113, 97, 119, 115, 101, 100, 114, 102, 116, 103, 121, 104 };
// q, a, w, s, e, d, r, f, t, g, y, h

const std::vector<int> known_keys_check = { 32, 122, 120, 99, 118, 98, 110 };
// SPACEBAR, z, x, c, v, b, n

struct camera_state
{
    PSEyeVideoCapture *camera;
    std::string identifier;
	std::chrono::high_resolution_clock::time_point last_ticks;
	int last_frames;

};

int main(int, char**)
{
	concurrency::concurrent_vector<camera_state> camera_states;
	int frame_rate_init = 40;

    // Open all available cameras (up to 30 max)
	for (int camera_index = 0; camera_index < 30; ++camera_index)
	{
        PSEyeVideoCapture *camera = new PSEyeVideoCapture(camera_index); // open the default camera

        if (camera->isOpened())
        {
            std::string identifier = camera->getUniqueIndentifier();

			std::cout << "Enter initial frame rate for  camera " << identifier << ": " ;
			std::cin >> frame_rate_init;
			
			if (camera->get(CV_CAP_PROP_FPS) != frame_rate_init)
			{
				camera->set(CV_CAP_PROP_FPS, frame_rate_init);
			}

			auto last_ticks = std::chrono::high_resolution_clock::now();
			int last_frames = 0;

            camera_states.push_back({ camera, identifier, last_ticks, last_frames });
        }
        else
        {
            delete camera;
        }
    }
	std::cout << "=========CONTROLS=========\n" << " + | - | value | Variable\n" << "___|___|_______|__________\n"
		<< " q | a |   z   | Exposure \n"
		<< " w | s |   x   | Contrast \n"
		<< " e | d |   c   | Gain \n"
		<< " r | f |   v   | Hue \n"
		<< " t | g |   b   | Sharpness \n"
		<< " y | h |   n   | Fame Rate \n"
		<< "Check the calculated frame rate with the space bar and close cameras with escape \n"
		;
    // Create a window for each opened camera
	concurrency::parallel_for_each(
        camera_states.begin(), 
        camera_states.end(),
		[&camera_states](camera_state &state) {
            cv::namedWindow(state.identifier.c_str(), 1);

    //bool bKeepRunning = camera_states.size() > 0;
	bool bKeepRunning = state.camera->isOpened();
    while (bKeepRunning)
    {
        // Render each camera frame in it's own window
        cv::Mat frame;
		(*state.camera) >> frame; // get a new frame from camera

        if (!frame.empty())
        {
            imshow(state.identifier.c_str(), frame);
			state.last_frames++;
        }

        int wk = cv::waitKey(10);

        if (wk == 27)  // Escape
        {
            bKeepRunning = false;
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

			// y/h for +/- frame rate
			// For CL_Eye, don't know
			if ((wk == 121) || (wk == 104))
			{
				cap_prop = CV_CAP_PROP_FPS;
				prop_str = "CV_CAP_PROP_FPS";
				val_diff = (wk == 121) ? 10 : -10;
			}

            double val = state.camera->get(cap_prop);
            std::cout << state.identifier << ": Value of " << prop_str << " was " << val << std::endl;

			switch (cap_prop)
			{
			case CV_CAP_PROP_FPS:
				if (val == 2) { if (val_diff > 0) val_diff = 1; else val_diff = 0; }
				else if (val == 3) { if (val_diff > 0) val_diff = 2; else val_diff = -1; }
				else if (val == 5) { if (val_diff > 0) val_diff = 3; else val_diff = -2; }
				else if (val == 8) { if (val_diff > 0) val_diff = 2; else val_diff = -3; }
				else if (val == 10) { if (val_diff > 0) val_diff = 5; else val_diff = -2; }
				else if (val == 15) { if (val_diff > 0) val_diff = 5; else val_diff = -5; }
				else if (val == 20) { if (val_diff > 0) val_diff = 5; else val_diff = -5; }
				else if (val == 25) { if (val_diff > 0) val_diff = 5; else val_diff = -5; }
				else if (val == 30) { if (val_diff < 0) { val_diff = -5; } }
				else if (val == 60) { if (val_diff > 0) { val_diff = 15; } }
				else if (val == 75) { if (val_diff > 0) val_diff = 8; else val_diff = -15; }
				else if (val == 83) { if (val_diff < 0) val_diff = -8; }
			}

			val += val_diff;
            state.camera->set(cap_prop, val);

            val = state.camera->get(cap_prop);
            std::cout << state.identifier << ": Value of " << prop_str << " changed by " << val_diff << " and is now " << val << std::endl;

        }
		else if (std::find(known_keys_check.begin(), known_keys_check.end(), wk) != known_keys_check.end())
		{
			int cap_prop = CV_CAP_PROP_FPS;
			std::string prop_str("CV_CAP_PROP_FPS");

			// SPACEBAR to check frame rate
			if ((wk == 32) || (wk == 110))
			{
				cap_prop = CV_CAP_PROP_FPS;
				prop_str = "CV_CAP_PROP_FPS";
			}

			// Z to check exposure
			if ((wk == 122))
			{
				cap_prop = CV_CAP_PROP_EXPOSURE;
				prop_str = "CV_CAP_PROP_EXPOSURE";
			}

			// X to check contrast
			if ((wk == 120))
			{
				cap_prop = CV_CAP_PROP_CONTRAST;
				prop_str = "CV_CAP_PROP_CONTRAST";
			}

			// C to check gain
			if ((wk == 99))
			{
				cap_prop = CV_CAP_PROP_GAIN;
				prop_str = "CV_CAP_PROP_GAIN";
			}

			// V to check hue
			if ((wk == 118))
			{
				cap_prop = CV_CAP_PROP_HUE;
				prop_str = "CV_CAP_PROP_HUE";
			}

			// B to check sharpness
			if ((wk == 98))
			{
				cap_prop = CV_CAP_PROP_SHARPNESS;
				prop_str = "CV_CAP_PROP_SHARPNESS";
			}

			double val = state.camera->get(cap_prop);
			auto now_ticks = std::chrono::high_resolution_clock::now();
			switch (wk)
			{
			case 32:
				std::cout << state.identifier << ": Fame rate is set to " << val << " and was actually " 
					<< (1000 * state.last_frames / (float(std::chrono::duration<double, std::milli>(now_ticks - state.last_ticks).count()))) << " fps" << std::endl;
				state.last_ticks = now_ticks;
				state.last_frames = 0;
				break;
			default:
				std::cout << state.identifier << ": Value of " << prop_str << " is " << val << std::endl;
			}

		}
        else if (wk > 0)
        {
            std::cout << "Unknown key has code " << wk << std::endl;
        }
    }

	delete state.camera;
	cvDestroyWindow(state.identifier.c_str());

    }
    );
    camera_states.clear();

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
