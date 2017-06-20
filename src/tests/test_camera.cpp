#include "PSEyeVideoCapture.h"
#include "ClientConstants.h"
#include "opencv2/opencv.hpp"
#include <algorithm>
#include <vector>
#include <chrono>

const std::vector<int> known_keys = {113, 97, 119, 115, 101, 100, 114, 102, 116, 103, 121, 104, 117, 106};
// q, a, w, s, e, d, r, f, t, g, y, h, u, j

const std::vector<int> known_keys_check = { 32, 122, 120, 99, 118, 98, 110, 109 };
// SPACEBAR, z, x, c, v, b, n, m

struct camera_state
{
    PSEyeVideoCapture *camera;
    std::string identifier;
	std::chrono::high_resolution_clock::time_point last_ticks;
	int last_frames;
};

int main(int, char**)
{
    std::vector<camera_state> camera_states;
	int frame_rate_init = 40;
	int frame_width_init = 640;

	std::cout << "=========CONTROLS=========\n"
		<< " + | - | value | Variable\n" << "___|___|_______|__________\n"
		<< " q | a |   z   | Exposure \n"
		<< " w | s |   x   | Contrast \n"
		<< " e | d |   c   | Gain \n"
		<< " r | f |   v   | Hue \n"
		<< " t | g |   b   | Sharpness \n"
		<< " y | h |   n   | Fame Rate \n"
		<< " u | j |   m   | Fame Width \n"
		<< "The space bar calculates the frame rate from the processed frames.\n"
		<< "The escape key will close the cameras.\n"
		<< "(focus must be on one of the camera windows to apply)\n"
		<< std::endl;

	std::cout << "Please enter the initial frame rate for the cameras\n"
		<< "available values range from 2 to 83 (but 83 is partially corrupt)\n"
		<< "(higher frame rates will need more USB bandwidth): \n";
	std::cin >> frame_rate_init;

	std::cout << "Please enter the initial frame width for the cameras:\n";
	std::cin >> frame_width_init;

    // Open all available cameras (up to 4 max)
	for (int camera_index = 0; camera_index < PSMOVESERVICE_MAX_TRACKER_COUNT; ++camera_index)
	{
        PSEyeVideoCapture *camera = new PSEyeVideoCapture(camera_index); // open the default camera

        if (camera->isOpened())
        {
            std::string identifier = camera->getUniqueIndentifier();

			if (camera->get(CV_CAP_PROP_FRAME_WIDTH) != frame_width_init) 
				camera->set(CV_CAP_PROP_FRAME_WIDTH, frame_width_init);

			if (camera->get(CV_CAP_PROP_FPS) != frame_rate_init) 
				camera->set(CV_CAP_PROP_FPS, frame_rate_init);

			auto last_ticks = std::chrono::high_resolution_clock::now();
			int last_frames = 0;

            camera_states.push_back({ camera, identifier, last_ticks, last_frames });
        }
        else
        {
            delete camera;
        }
    }
    
    // Create a window for each opened camera
    std::for_each(
        camera_states.begin(), 
        camera_states.end(),
        [&camera_states](camera_state &state) {
            cv::namedWindow(state.identifier.c_str(), 1);
        }
    );

    bool bKeepRunning = camera_states.size() > 0;
    while (bKeepRunning)
    {
        // Render each camera frame in it's own window
        std::for_each(
            camera_states.begin(),
            camera_states.end(),
            [&camera_states](camera_state &state) {
                cv::Mat frame;
                (*state.camera) >> frame; // get a new frame from camera

                if (!frame.empty())
                {
                    imshow(state.identifier.c_str(), frame);
					state.last_frames++;
                }
            }
        );

        int wk = cv::waitKey(1);

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

			// u/j for +/- frame width
			// For CL_Eye, don't know
			if ((wk == 117) || (wk == 106))
			{
				cap_prop = CV_CAP_PROP_FRAME_WIDTH;
				prop_str = "CV_CAP_PROP_FRAME_WIDTH";
				val_diff = (wk == 117) ? 1 : -1;
			}

            std::for_each(
                camera_states.begin(), 
                camera_states.end(),
                [&camera_states, &cap_prop, &prop_str, &val_diff](camera_state &state) {

                    double val = state.camera->get(cap_prop);
                    std::cout << state.identifier << ": Value of " << prop_str << " was " << val << std::endl;

					switch (cap_prop)
					{
					case CV_CAP_PROP_FRAME_WIDTH:
						if (val == 320) { if (val_diff > 0) val_diff = 320; else val_diff = 0; }
						else if (val == 640) { if (val_diff > 0) val_diff = 0; else val_diff = -320; }
						break;
					case CV_CAP_PROP_FPS:
						int frame_width = state.camera->get(CV_CAP_PROP_FRAME_WIDTH);
						if (frame_width == 640)
						{
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
						else
						{
							if (val == 2) { if (val_diff > 0) val_diff = 1; else val_diff = 0; }
							else if (val == 3) { if (val_diff > 0) val_diff = 2; else val_diff = -1; }
							else if (val == 5) { if (val_diff > 0) val_diff = 2; else val_diff = -2; }
							else if (val == 7) { if (val_diff > 0) val_diff = 3; else val_diff = -2; }
							else if (val == 10) { if (val_diff > 0) val_diff = 2; else val_diff = -3; }
							else if (val == 12) { if (val_diff > 0) val_diff = 3; else val_diff = -2; }
							else if (val == 15) { if (val_diff > 0) val_diff = 2; else val_diff = -3; }
							else if (val == 17) { if (val_diff > 0) val_diff = 13; else val_diff = -2; }
							else if (val == 30) { if (val_diff > 0) val_diff = 7; else val_diff = -13; }
							else if (val == 37) { if (val_diff > 0) val_diff = 3; else val_diff = -7; }
							else if (val == 40) { if (val_diff > 0) val_diff = 10; else val_diff = -3; }
							else if (val == 50) { if (val_diff > 0) val_diff = 10; else val_diff = -10; }
							else if (val == 60) { if (val_diff > 0) val_diff = 15; else val_diff = -10; }
							else if (val == 75) { if (val_diff > 0) val_diff = 15; else val_diff = -15; }
							else if (val == 90) { if (val_diff > 0) val_diff = 10; else val_diff = -15; }
							else if (val == 100) { if (val_diff > 0) val_diff = 25; else val_diff = -10; }
							else if (val == 125) { if (val_diff > 0) val_diff = 12; else val_diff = -25; }
							else if (val == 137) { if (val_diff > 0) val_diff = 13; else val_diff = -12; }
							else if (val == 150) { if (val_diff > 0) val_diff = 37; else val_diff = -13; }
							else if (val == 187) { if (val_diff > 0) val_diff = 18; else val_diff = -37; }
							else if (val == 205) { if (val_diff > 0) val_diff = 85; else val_diff = -18; }
							else if (val == 290) { if (val_diff > 0) val_diff = 0; else val_diff = -85; }
						}
						break;
					}

					val += val_diff;
                    state.camera->set(cap_prop, val);

                    val = state.camera->get(cap_prop);
                    std::cout << state.identifier << ": Value of " << prop_str << " changed by " << val_diff << " and is now " << val << std::endl;
                }
            );
        }
		else if (std::find(known_keys_check.begin(), known_keys_check.end(), wk) != known_keys_check.end())
		{
			int cap_prop = CV_CAP_PROP_FPS;
			std::string prop_str("CV_CAP_PROP_FPS");

			// SPACEBAR to check frame rate
			if (wk == 32)
			{
				cap_prop = CV_CAP_PROP_FPS;
				prop_str = "CV_CAP_PROP_FPS";
			}

			// Z to check exposure
			if (wk == 122)
			{
				cap_prop = CV_CAP_PROP_EXPOSURE;
				prop_str = "CV_CAP_PROP_EXPOSURE";
			}

			// X to check contrast
			if (wk == 120)
			{
				cap_prop = CV_CAP_PROP_CONTRAST;
				prop_str = "CV_CAP_PROP_CONTRAST";
			}

			// C to check gain
			if (wk == 99)
			{
				cap_prop = CV_CAP_PROP_GAIN;
				prop_str = "CV_CAP_PROP_GAIN";
			}

			// V to check hue
			if (wk == 118)
			{
				cap_prop = CV_CAP_PROP_HUE;
				prop_str = "CV_CAP_PROP_HUE";
			}

			// B to check sharpness
			if (wk == 98)
			{
				cap_prop = CV_CAP_PROP_SHARPNESS;
				prop_str = "CV_CAP_PROP_SHARPNESS";
			}

			// N to check frame rate
			if (wk == 110)
			{
				cap_prop = CV_CAP_PROP_FPS;
				prop_str = "CV_CAP_PROP_FPS";
			}

			// M to check frame width
			if (wk == 109)
			{
				cap_prop = CV_CAP_PROP_FRAME_WIDTH;
				prop_str = "CV_CAP_PROP_FRAME_WIDTH";
			}

			std::for_each(
				camera_states.begin(),
				camera_states.end(),
				[&camera_states, &cap_prop, &prop_str, &wk](camera_state &state) {
				
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
			);
		}
        else if (wk > 0)
        {
            std::cout << "Unknown key has code " << wk << std::endl;
        }
    }

    // Disconnect all active cameras
    std::for_each(
        camera_states.begin(), 
        camera_states.end(),
        [&camera_states](camera_state &state) {
            delete state.camera;
        }
    );
    camera_states.clear();

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
