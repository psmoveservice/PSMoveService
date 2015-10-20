#include "PSEyeVideoCapture.h"

#ifdef HAVE_PS3EYE
/**
 * Taken from the PS3EYEDriver OpenFrameworks example
 * written by Eugene Zatepyakin, MIT license
 **/

static const int ITUR_BT_601_CY = 1220542;
static const int ITUR_BT_601_CUB = 2116026;
static const int ITUR_BT_601_CUG = -409993;
static const int ITUR_BT_601_CVG = -852492;
static const int ITUR_BT_601_CVR = 1673527;
static const int ITUR_BT_601_SHIFT = 20;

static void
yuv422_to_bgr(const uint8_t *yuv_src, const int stride, uint8_t *dst, const int width, const int height)
{
    const int bIdx = 2;
    const int uIdx = 0;
    const int yIdx = 0;
    
    const int uidx = 1 - yIdx + uIdx * 2;
    const int vidx = (2 + uidx) % 4;
    int j, i;
    
#define _max(a, b) (((a) > (b)) ? (a) : (b))
#define _saturate(v) (uint8_t)((uint32_t)(v) <= 0xff ? v : v > 0 ? 0xff : 0)
    
    for (j = 0; j < height; j++, yuv_src += stride)
    {
        uint8_t* row = dst + (width * 3) * j;
        
        for (i = 0; i < 2 * width; i += 4, row += 6)
        {
            int u = (int)(yuv_src[i + uidx]) - 128;
            int v = (int)(yuv_src[i + vidx]) - 128;
            
            int ruv = (1 << (ITUR_BT_601_SHIFT - 1)) + ITUR_BT_601_CVR * v;
            int guv = (1 << (ITUR_BT_601_SHIFT - 1)) + ITUR_BT_601_CVG * v + ITUR_BT_601_CUG * u;
            int buv = (1 << (ITUR_BT_601_SHIFT - 1)) + ITUR_BT_601_CUB * u;
            
            int y00 = _max(0, (int)(yuv_src[i + yIdx]) - 16) * ITUR_BT_601_CY;
            row[2-bIdx] = _saturate((y00 + buv) >> ITUR_BT_601_SHIFT);
            row[1]      = _saturate((y00 + guv) >> ITUR_BT_601_SHIFT);
            row[bIdx]   = _saturate((y00 + ruv) >> ITUR_BT_601_SHIFT);
            
            int y01 = _max(0, (int)(yuv_src[i + yIdx + 2]) - 16) * ITUR_BT_601_CY;
            row[5-bIdx] = _saturate((y01 + buv) >> ITUR_BT_601_SHIFT);
            row[4]      = _saturate((y01 + guv) >> ITUR_BT_601_SHIFT);
            row[3+bIdx] = _saturate((y01 + ruv) >> ITUR_BT_601_SHIFT);
        }
    }
}
#undef _max
#undef _saturate
#endif

bool PSEyeVideoCapture::open(int index)
{
    if (isOpened()) release();

	// Try to open a PS3EYE-specific camera capture

#if !defined(CV_VERSION_EPOCH) || (CV_VERSION_EPOCH > 2)
    /* TODO: Consider using IVideoCapture
    icap = IVideoCapture_create(index);
    if (!icap.empty())
        return true;
    */
    cap.reset(pseyeCreateCameraCapture(index));
#else  // OpenCV2
    cap = pseyeCreateCameraCapture(index);
#endif
    
	// PS3EYE-specific camera capture if available, else default OpenCV object
    return isOpened() ? isOpened() : cv::VideoCapture::open(index);
}

CvCapture* PSEyeVideoCapture::pseyeCreateCameraCapture(int index)
{
    int pref = (index / 100) * 100;
    index -= pref;
    CvCapture *capture = 0; // no match
    
    switch (pref)
    {
        default:
            if (pref)
            {
                // Passed in a non-zero preference
                // that didn't match any switch cases
                break;
            }
            
#ifdef HAVE_CLEYE
			/*
        case PSEYE_CAP_CLMULTI:
            if (!capture)
            {
                capture = pseyeCreateCameraCapture_CLMULTI(index);
            }
            if (pref)
            {
                break;
            }
			
            
        case PSEYE_CAP_CLEYE:
            if (!capture)
            {
                capture = pseyeCreateCameraCapture_CLEYE(index);
            }
            if (pref)
            {
                break;
            }
			*/
#endif //HAVE_CLEYE

#ifdef HAVE_PS3EYE
        case PSEYE_CAP_PS3EYE:
            if (!capture)
            {
                capture = pseyeCreateCameraCapture_PS3EYE(index);
            }
            if (pref)
            {
                break;
            }
#endif //HAVE_PS3EYE
    }
    
    return capture;
}

#ifdef HAVE_CLEYE
/*
CvCapture* PSEyeVideoCapture::pseyeCreateCameraCapture_CLMULTI(int index)
{
    PSEEYECaptureCAM_CLMULTI* capture = new PSEEYECaptureCAM_CLMULTI;
    try
    {
        if( capture->open( index ))
            return (CvCapture*)capture;
    }
    catch(...)
    {
        delete capture;
        throw;
    }
    delete capture;
    return 0;
}


CvCapture* PSEyeVideoCapture::pseyeCreateCameraCapture_CLEYE(int index)
{
    PSEEYECaptureCAM_CLEYE* capture = new PSEEYECaptureCAM_CLEYE;
    try
    {
        if( capture->open( index ))
            return (CvCapture*)capture;
    }
    catch(...)
    {
        delete capture;
        throw;
    }
    delete capture;
    return 0;
}
*/
#endif

#ifdef HAVE_PS3EYE
CvCapture* PSEyeVideoCapture::pseyeCreateCameraCapture_PS3EYE(int index)
{
    PSEEYECaptureCAM_PS3EYE* capture = new PSEEYECaptureCAM_PS3EYE;
    try
    {
        if( capture->open( index ))
            return (CvCapture*)capture;
    }
    catch(...)
    {
        delete capture;
        throw;
    }
    delete capture;
    return 0;
}
#endif


/*
 * PSEEYECaptureCAM_CLMULTI
 */
#ifdef HAVE_CLEYE
/*
PSEEYECaptureCAM_CLMULTI::PSEEYECaptureCAM_CLMULTI():
frame(NULL)
{

}

PSEEYECaptureCAM_CLMULTI::~PSEEYECaptureCAM_CLMULTI()
{
    close();
}

void PSEEYECaptureCAM_CLMULTI::close()
{
}

// Initialize camera input
bool PSEEYECaptureCAM_CLMULTI::open( int _index )
{
    return false;
}

bool PSEEYECaptureCAM_CLMULTI::grabFrame()
{
    return false;
}

IplImage* PSEEYECaptureCAM_CLMULTI::retrieveFrame(int)
{
    return frame;
}

double PSEEYECaptureCAM_CLMULTI::getProperty( int property_id ) const
{
    return 0;
}
bool PSEEYECaptureCAM_CLMULTI::setProperty( int property_id, double value )
{
    return false;
}
*/

/*
 * PSEEYECaptureCAM_CLEYE
 */

/*
PSEEYECaptureCAM_CLEYE::PSEEYECaptureCAM_CLEYE():
frame(NULL)
{
    
}

 PSEEYECaptureCAM_CLEYE::~PSEEYECaptureCAM_CLEYE()
 {
 close();
 }

void PSEEYECaptureCAM_CLEYE::close()
{
}

// Initialize camera input
bool PSEEYECaptureCAM_CLEYE::open( int _index )
{
    return false;
}

bool PSEEYECaptureCAM_CLEYE::grabFrame()
{
    return false;
}

IplImage* PSEEYECaptureCAM_CLEYE::retrieveFrame(int)
{
    return frame;
}

double PSEEYECaptureCAM_CLEYE::getProperty( int property_id ) const
{
    return 0;
}
bool PSEEYECaptureCAM_CLEYE::setProperty( int property_id, double value )
{
    return false;
}
*/
#endif

/*
 * PSEEYECaptureCAM_PS3EYE
 */
#ifdef HAVE_PS3EYE
PSEEYECaptureCAM_PS3EYE::PSEEYECaptureCAM_PS3EYE():
frame(NULL)
{
    //CoInitialize(NULL);
    
    // Enumerate libusb devices
    std::vector<ps3eye::PS3EYECam::PS3EYERef> devices = ps3eye::PS3EYECam::getDevices();
    std::cout << "ps3eye::PS3EYECam::getDevices() found " << devices.size() << " devices." << std::endl;
    if (devices.size() > 0) {
        eye = devices[0];
    }
}


PSEEYECaptureCAM_PS3EYE::~PSEEYECaptureCAM_PS3EYE()
{
    close();
    
    // eye is a smart pointer to a class.
    // When it goes out of scope, it should trigger the class destructor.
    
    //CoUninitialize();
}

void PSEEYECaptureCAM_PS3EYE::close()
{
    cvReleaseImage( &frame );
}

// Initialize camera input
bool PSEEYECaptureCAM_PS3EYE::open( int _index )
{
    if (eye && eye->init(640, 480, 60))
    {
        // Change any default settings here
        
        frame = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
        
        eye->start();
        
        return true;
    }
    else {
        return false;
    }
}

bool PSEEYECaptureCAM_PS3EYE::grabFrame()
{
    // TODO: I don't like blocking until we get a frame.
    while (!eye->isNewFrame())
    {
        ps3eye::PS3EYECam::updateDevices();
    }
    return true;
}

IplImage* PSEEYECaptureCAM_PS3EYE::retrieveFrame(int)
{
    const unsigned char *new_pixels = eye->getLastFramePointer();
    
    widthStep = eye->getRowBytes();
    width = eye->getWidth();
    height = eye->getHeight();
    size = widthStep * height;
    
    // Convert pixels from camera to OpenCV BGR image
    unsigned char *cvpixels;
    cvGetRawData(frame, &cvpixels, 0, 0); // cvpixels becomes low-level access to frame
    if (new_pixels != NULL)
    {
        // Simultaneously copy from new_pixels to cvpixels and convert colorspace.
        yuv422_to_bgr(new_pixels, widthStep, cvpixels, width, height);
        
        // This seems like an opportunity for optimization.
        // Maybe we can use OpenCV tools to convert the memory format
        // (see cvInitImageHeader and cvSetData)
        // Then use OpenCV tools to convert the colorspace. e.g.,
        // cvtColor(src,dst,CV_YUV2BGR_YUY2);
    }
    return frame;
}

double PSEEYECaptureCAM_PS3EYE::getProperty( int property_id ) const
{
    switch( property_id )
    {
        case CV_CAP_PROP_BRIGHTNESS:
            //return eye->getBrightness();
            return 0;
        case CV_CAP_PROP_CONTRAST:
            // default 37
            // Seems to be related to gain. Higher value is noisier.
            return eye->getContrast() / 255;
        case CV_CAP_PROP_EXPOSURE:
            // Default 120
            return (double)eye->getExposure() / 255;
        case CV_CAP_PROP_FPS:
            return (double)eye->getFrameRate();
        case CV_CAP_PROP_FRAME_HEIGHT:
            return eye->getHeight();
        case CV_CAP_PROP_FRAME_WIDTH:
            return eye->getWidth();
        case CV_CAP_PROP_GAIN:
            // Default 20
            return double(eye->getGain()) / 255;
        case CV_CAP_PROP_HUE:
            // Default 143
            return double(eye->getHue()) * 180/255;
    }
    return 0;
}
bool PSEEYECaptureCAM_PS3EYE::setProperty( int property_id, double value )
{
    if (!eye)
    {
        return false;
    }
    switch (property_id)
    {
        case CV_CAP_PROP_BRIGHTNESS:
            return false;
            //eye->setBrightness(round(value));
            // While the internal value updates correctly, it doesn't seem to
            // be affecting brightness. It's affecting hue.
        case CV_CAP_PROP_CONTRAST:
            // Seems to be affecting gain, not contrast
            eye->setContrast(round(255 * value));
        case CV_CAP_PROP_EXPOSURE:
            eye->setExposure(round((255*value)));
        case CV_CAP_PROP_FPS:
            return false; //TODO: Modifying FPS probably requires resetting the camera
        case CV_CAP_PROP_FRAME_HEIGHT:
            return false; //TODO: Modifying frame size probably requires resetting the camera
        case CV_CAP_PROP_FRAME_WIDTH:
            return false;
        case CV_CAP_PROP_GAIN:
            eye->setGain(round(255 * value));
        case CV_CAP_PROP_HUE:  // input expected 0-180; output expects 0-255
            eye->setHue(255 * value / 180);
    }
    return true;
}
#endif